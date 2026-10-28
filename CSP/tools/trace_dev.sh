#!/bin/bash
# Load a DebugTrace ELF into RAM, run it in development boot (BOOT1 = 1) for
# N seconds, interrupt, dump the trace ring (header and records) and detach:
# one GDB session through the ST-LINK GDB server (SWD, AP1, connect under
# reset). No external loader: the NOR flash is never written. A breakpoint on
# HAL_BSEC_OTP_Program resets the target before any OTP write.
# After the GDB server has shut down, the VCP is captured for AFTER seconds
# more: output there means the target kept running without the debugger.
#
# Usage: CSP/tools/trace_dev.sh <file.elf> [seconds] [after-seconds]
# Environment: STLINK_SN (required) the serial number of the board's ST-LINK
# (STM32_Programmer_CLI -l lists it); CUBEIDE (required) the STM32CubeIDE
# installation directory (its GDB server, STM32CubeProgrammer and GNU tools
# are used); VCP the board's virtual COM port (default: the Linux
# /dev/serial/by-id entry of that ST-LINK); PORT the GDB port (61234).
# Output: $OUT (default: a new temporary directory): ring.bin (struct
# csp_trace_ring), ring.txt (header and DWT as GDB printed them), vcp.log,
# vcp_after.log, gdb.log, gdbserver.log. GDB_EXTRA: a GDB command file run after
# the ring readout, its output logged to ring.txt. RING_POLICY=stop: the ring
# keeps the first records from boot and drops the rest (set by the debugger
# before the first task starts; default: the build's policy, overwrite).
# CSP_TOOLS: a CSP4CMSIS checkout with the ring tools (tools/trace,
# trace/FreeRTOS_ring.scvd), e.g. the submodule Lib/CSP4CMSIS; when set,
# the dump is then analysed in $OUT:
# ring2log.py (events.log, ring_report.txt), decode.py with eventlist
# (events.csv, summary.txt, stable.csv, trace.json; EVENTLIST as in
# tools/trace/README.md) and csp_trace.py (conf/) with the interrupt writers
# of ISR_WRITERS (default: CSP/tools/isr_writers.json); PROBCLI set: proved.
set -euo pipefail

elf=$(realpath "${1:?usage: $0 <file.elf> [seconds] [after-seconds]}")
sn=${STLINK_SN:?set STLINK_SN to the ST-LINK serial number}
secs=${2:-10}
after=${3:-15}
port=${PORT:-61234}
ide=${CUBEIDE:?set CUBEIDE to the STM32CubeIDE installation directory}
out=${OUT:-$(mktemp -d "${TMPDIR:-/tmp}/trace_dev.XXXXXX")}
isr_writers=$(realpath "${ISR_WRITERS:-$(dirname "$0")/isr_writers.json}")
case "${RING_POLICY:-}" in
  "") policy_cmd= ;;
  stop) policy_cmd='set var csp_trace_ring.header.policy = 1
printf "trace_dev: ring policy stop (from boot)\n"' ;;
  *) echo "trace_dev: RING_POLICY must be stop or unset"; exit 2 ;;
esac
mkdir -p "$out"

plugins=$ide/plugins
gdbserver=$(ls -d "$plugins"/*stlink-gdb-server*/tools/bin)/ST-LINK_gdbserver
cubeprog=$(ls -d "$plugins"/*cubeprogrammer*/tools/bin)
gdb=$(ls -d "$plugins"/*gnu-tools-for-stm32.14*/tools/bin)/arm-none-eabi-gdb
tty=${VCP:-$(ls /dev/serial/by-id/*STLINK*_"$sn"-if0*)}

cat_pid= gs_pid= gdb_pid=
cleanup() {
  for p in $gdb_pid $gs_pid $cat_pid; do kill "$p" 2>/dev/null || true; done
  wait 2>/dev/null || true
}
trap cleanup EXIT

stty -F "$tty" 115200 cs8 -cstopb -parenb raw -echo -ixon -ixoff
cat "$tty" > "$out/vcp.log" &
cat_pid=$!

"$gdbserver" -p "$port" -l 1 -d -k -m 1 -i "$sn" -cp "$cubeprog" -f "$out/gdbserver.log" \
  > "$out/gdbserver.out" 2>&1 &
gs_pid=$!
for _ in $(seq 50); do
  ss -ltn | grep -q ":$port " && break
  sleep 0.2
done

cat > "$out/trace.gdb" <<EOF
set pagination off
set confirm off
target remote localhost:$port
monitor reset
load
break HAL_BSEC_OTP_Program
commands
  printf "trace_dev: HAL_BSEC_OTP_Program reached; resetting target, OTP not written\n"
  monitor reset
  quit 2
end
tbreak npu_cache_enable
continue
delete
printf "trace_dev: Fuse_Programming returned without programming OTP\n"
$policy_cmd
printf "trace_dev: running\n"
continue
printf "trace_dev: interrupted\n"
set logging file $out/ring.txt
set logging overwrite on
set logging redirect off
set logging enabled on
print/x csp_trace_ring.header
print SystemCoreClock
printf "DWT_CTRL 0x%08x CYCCNT 0x%08x DEMCR 0x%08x\n", *(unsigned *)0xE0001000, *(unsigned *)0xE0001004, *(unsigned *)0xE000EDFC
printf "MPU_TYPE 0x%08x MPU_CTRL 0x%08x MPU_RNR 0x%08x MAIR0 0x%08x MAIR1 0x%08x\n", *(unsigned *)0xE000ED90, *(unsigned *)0xE000ED94, *(unsigned *)0xE000ED98, *(unsigned *)0xE000EDC0, *(unsigned *)0xE000EDC4
printf "MPU region RNR: RBAR 0x%08x RLAR 0x%08x\n", *(unsigned *)0xE000ED9C, *(unsigned *)0xE000EDA0
print csp4cmsis_last_fatal_error
${GDB_EXTRA:+source $(realpath "$GDB_EXTRA")}
set logging enabled off
dump binary memory $out/ring.bin &csp_trace_ring ((char *)&csp_trace_ring) + sizeof(csp_trace_ring)
printf "trace_dev: ring dumped\n"
detach
EOF
# The script's plain "continue" returns when GDB gets SIGINT (as for Ctrl-C):
# GDB then interrupts the target and runs the rest of the script.
# A background job of a non-interactive shell starts with SIGINT ignored:
# env --default-signal restores it for GDB.
env --default-signal=INT "$gdb" -batch -nx -x "$out/trace.gdb" "$elf" > "$out/gdb.log" 2>&1 &
gdb_pid=$!
for _ in $(seq 300); do
  grep -q "^trace_dev: running" "$out/gdb.log" && break
  kill -0 "$gdb_pid" 2>/dev/null || break
  sleep 0.1
done
grep -q "^trace_dev: running" "$out/gdb.log" || { cat "$out/gdb.log"; echo "trace_dev: target not released"; exit 1; }
sleep "$secs"
kill -INT "$gdb_pid"
wait "$gdb_pid" || { cat "$out/gdb.log"; echo "trace_dev: GDB failed"; exit 1; }
cat "$out/gdb.log"

gdb_pid=
kill "$gs_pid" 2>/dev/null || true
wait "$gs_pid" 2>/dev/null || true
gs_pid=
kill "$cat_pid" 2>/dev/null || true
wait "$cat_pid" 2>/dev/null || true
cat "$tty" > "$out/vcp_after.log" &
cat_pid=$!
echo "trace_dev: GDB server shut down at $(date +%T); VCP for $after s more"
sleep "$after"
echo "trace_dev: VCP during the run ($out/vcp.log):"
cat "$out/vcp.log"
kill "$cat_pid" 2>/dev/null || true
wait "$cat_pid" 2>/dev/null || true
cat_pid=
echo "trace_dev: VCP after the GDB server shut down ($out/vcp_after.log, $(wc -c < "$out/vcp_after.log") bytes):"
cat "$out/vcp_after.log"

[ -n "${CSP_TOOLS:-}" ] || exit 0
tools=$CSP_TOOLS/tools/trace
echo "trace_dev: analysing with $CSP_TOOLS, interrupt writers from $isr_writers"
python3 -I "$tools/ring2log.py" "$out/ring.bin" -o "$out/events.log" --report "$out/ring_report.txt"
python3 -I "$tools/decode.py" -a "$elf" -I "$CSP_TOOLS/trace/FreeRTOS_ring.scvd" --nm "$(dirname "$gdb")/arm-none-eabi-nm" \
  "$out/events.log" -o "$out/events.csv" --summary "$out/summary.txt" --stable "$out/stable.csv" --perfetto "$out/trace.json"
cat "$out/ring_report.txt" "$out/summary.txt"
rm -rf "$out/conf"
python3 -I "$tools/csp_trace.py" "$out/events.csv" --stable "$out/stable.csv" --out "$out/conf" --mutate \
  --isr-writers "$isr_writers"
[ -n "${PROBCLI:-}" ] || exit 0
PROBCLI=$PROBCLI python3 -I "$tools/csp_trace.py" --prove "$out/conf"
