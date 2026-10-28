#!/bin/bash
# Load an ELF into RAM and run it in development boot (BOOT1 = 1) through the
# ST-LINK GDB server (SWD, AP1, connect under reset), capturing the VCP output.
# No external loader is passed to the GDB server, so the NOR flash is never written.
# A breakpoint on HAL_BSEC_OTP_Program resets the target before any OTP write.
#
# Usage: CSP/tools/run_dev.sh <file.elf> [seconds]
# Environment: STLINK_SN (required) the serial number of the board's ST-LINK
# (STM32_Programmer_CLI -l lists it); CUBEIDE (required) the STM32CubeIDE
# installation directory (its GDB server, STM32CubeProgrammer and GNU tools
# are used); VCP the board's virtual COM port (default: the Linux
# /dev/serial/by-id entry of that ST-LINK); PORT the GDB port (61234).
# Output: $OUT (default: a new temporary directory) holds vcp.log, gdb.log, gdbserver.log.
set -euo pipefail

elf=$(realpath "${1:?usage: $0 <file.elf> [seconds]}")
sn=${STLINK_SN:?set STLINK_SN to the ST-LINK serial number}
secs=${2:-30}
port=${PORT:-61234}
ide=${CUBEIDE:?set CUBEIDE to the STM32CubeIDE installation directory}
out=${OUT:-$(mktemp -d "${TMPDIR:-/tmp}/run_dev.XXXXXX")}
mkdir -p "$out"

plugins=$ide/plugins
gdbserver=$(ls -d "$plugins"/*stlink-gdb-server*/tools/bin)/ST-LINK_gdbserver
cubeprog=$(ls -d "$plugins"/*cubeprogrammer*/tools/bin)
gdb=$(ls -d "$plugins"/*gnu-tools-for-stm32.14*/tools/bin)/arm-none-eabi-gdb
tty=${VCP:-$(ls /dev/serial/by-id/*STLINK*_"$sn"-if0*)}

pids=()
cleanup() { for p in "${pids[@]}"; do kill "$p" 2>/dev/null || true; done; wait 2>/dev/null || true; }
trap cleanup EXIT

stty -F "$tty" 115200 cs8 -cstopb -parenb raw -echo -ixon -ixoff
cat "$tty" > "$out/vcp.log" &
pids+=($!)

"$gdbserver" -p "$port" -l 1 -d -k -m 1 -i "$sn" -cp "$cubeprog" -f "$out/gdbserver.log" \
  > "$out/gdbserver.out" 2>&1 &
pids+=($!)
for _ in $(seq 50); do
  ss -ltn | grep -q ":$port " && break
  sleep 0.2
done

cat > "$out/run.gdb" <<EOF
set pagination off
set confirm off
target remote localhost:$port
monitor reset
load
break HAL_BSEC_OTP_Program
commands
  printf "run_dev: HAL_BSEC_OTP_Program reached; resetting target, OTP not written\n"
  monitor reset
  quit 2
end
tbreak npu_cache_enable
continue
delete
printf "run_dev: Fuse_Programming returned without programming OTP\n"
detach
EOF
"$gdb" -batch -nx -x "$out/run.gdb" "$elf" 2>&1 | tee "$out/gdb.log"

sleep "$secs"
echo "run_dev: VCP log ($secs s after release) in $out/vcp.log"
cat "$out/vcp.log"
