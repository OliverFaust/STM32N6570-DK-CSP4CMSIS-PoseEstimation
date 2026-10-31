# STM32N6570-DK-CSP4CMSIS-PoseEstimation

[STM32N6570-DK-CSP4CMSIS-PoseEstimation](https://github.com/OliverFaust/STM32N6570-DK-CSP4CMSIS-PoseEstimation),
branch `csp4cmsis`, is an unofficial fork of STMicroelectronics'
[x-cube-n6-ai-multi-pose-estimation](https://github.com/STMicroelectronics/x-cube-n6-ai-multi-pose-estimation)
v2.2.0. It replaces the application's FreeRTOS threads with a
[CSP4CMSIS](https://github.com/OliverFaust/CSP4CMSIS) process network and adds a
RAM trace of that network. It is not affiliated with, reviewed or endorsed by
STMicroelectronics. ST's README follows [below](#sts-readme), unchanged.

## What this fork adds

- **A CSP process network** (`CSP/application.cpp`) in place of the four
  threads of ST's `Src/app.c`. Six CSP4CMSIS processes, all at one priority,
  form the pipeline isp → nn → nn_2 → pp → dp, with an NPU concentrator
  process beside nn and nn_2:
  - The camera's vsync interrupt writes a one-slot KeepNewest channel. isp
    reads it, updates the camera's ISP and sends a time-stamped frame message
    to nn.
  - nn and nn_2 both request the NPU from the concentrator, which serves the
    two request channels with a fair ALT (`fairSelect()`). Only nn_2's request
    runs the inference (`NN_Execute_Inference()`); nn's is answered without
    one. After its reply, nn passes the frame on to nn_2. After its reply,
    nn_2 sends the inference time and the frame period to pp.
  - pp runs the post-processing and sends the result to dp through a one-slot
    KeepNewest channel. dp draws it on the display and every 5 s prints a
    stats line on the console (VCP).
  - The other seven channels (isp → nn → nn_2 → pp and the request and reply
    channels between nn, nn_2 and the concentrator) are rendezvous channels.
- **Static allocation only.** FreeRTOS already runs without a heap in ST's
  v2.2.0. CSP4CMSIS creates every RTOS object statically, and newlib's heap
  (used by the C++ runtime) is bounded: `_sbrk()` fails beyond `_eheap`.
- **DebugTrace**, a second build configuration: CSP4CMSIS's RAM ring trace
  backend records every CSP event and the RTOS task switches into a 256 KB
  ring (16384 records with time stamps from DWT_CYCCNT). The debugger reads
  the ring out, and CSP4CMSIS's tools decode it and check it against the
  library's formal models.
- **Tools** (`CSP/tools/`) for running and tracing in development boot.
- **The network explained:** [`CSP/README.md`](CSP/README.md) describes the
  design, the measured behaviour, stack occupancy and troubleshooting.

The library is the git submodule `Lib/CSP4CMSIS`, on CSP4CMSIS's
`dev/tracing` branch. That branch is not a release: its API is identical to
CSP4CMSIS 3.1.0, but the trace format and the ring backend are experimental
and may change.

ST's files changed by this fork, and why:

| File | Change |
|:-----|:-------|
| `Src/app.c` | The thread bodies become stage functions the processes call; the vsync callback signals the network; `app_run()` starts the network instead of the four threads |
| `Inc/app.h` | `display_info_t` (and the tracker types it uses) moved here from `app.c`: it is the message from pp to dp |
| `Src/main.c` | `osKernelInitialize()` / `osKernelStart()` (CMSIS-RTOS2) around the main thread |
| `Inc/FreeRTOSConfig.h` | `configUSE_OS2_THREAD_FLAGS 1` (CSP4CMSIS uses thread flags); in DebugTrace only, the tick hook and the task hooks of the trace |
| `Lib/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c` | Now built. `IS_IRQ_MASKED()` also tests BASEPRI on Armv8.1-M (Cortex-M55); see the notice in the file |
| `Gcc/STM32N657xx.ld`, `Gcc/Src/syscalls.c` | Newlib heap of 4 KB, ending at `_eheap`; a bounded `_sbrk()` |
| `STM32CubeIDE/.project`, `.cproject` | `CSP/`, the library and `cmsis_os2.c` in the build; C++ settings; the DebugTrace configuration |

This branch supports the **STM32CubeIDE build only**. ST's EWARM project and
Makefile are still there, unchanged, but they do not build the CSP4CMSIS
sources.

## What you need

- An STM32N6570-DK (MB1939) with the IMX335 camera (MB1854B), powered through
  the ST-LINK USB-C connector (CN6), as in ST's README.
- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) 2.1.0
  (it includes GNU Tools for STM32 14.3.rel1).
- The STM32CubeN6 drivers (v1.3.0): they come with ST's repository in
  `STM32Cube_FW_N6/`, under ST's licence terms (`LICENSE.md`). You do not
  need to install the STM32CubeN6 package separately for this build. By using
  them you accept ST's terms.
- The pose estimation model, generated and in the board's external flash
  **before** you run anything: see "Model" below.
- For tracing: Python 3, CMSIS-Toolbox `eventlist` (to decode), and
  optionally [ProB](https://prob.hhu.de/) `probcli` (to check against the
  models); see `Lib/CSP4CMSIS/tools/trace/README.md`.

## Model

Like ST's v2.2.0, this repository contains no network code and no weights.
They are generated from the YOLOv8 pose model, which is AGPL-3.0, so read
ST's licence note in the instructions first. Follow ST's instructions,
[Download and deploy the model](Doc/Download-and-deploy-model.md):

1. Download the model.
2. Generate the network code and weights with ST Edge AI 3.0.0
   (`Model/generate-n6-model.sh`).
3. Program `Model/network_data.hex` into the external flash with
   STM32CubeProgrammer.

The fork changes nothing in `Model/`. The application reads the weights
from external flash at run time, so they must be there before the first run.
Generated files stay local (do not commit them).

## Clone and build

```bash
git clone --recurse-submodules -b csp4cmsis https://github.com/OliverFaust/STM32N6570-DK-CSP4CMSIS-PoseEstimation.git
cd STM32N6570-DK-CSP4CMSIS-PoseEstimation
```

(An existing clone: `git submodule update --init`.)

Generate the model (above), then open `STM32CubeIDE/.project` in
STM32CubeIDE and build **Debug** or **DebugTrace**. Or build without the IDE:

```bash
<STM32CubeIDE>/headless-build.sh -data <new workspace directory> \
  -import STM32CubeIDE -build x-cube-n6-ai-multi-pose-estimation/Debug
```

(`-build x-cube-n6-ai-multi-pose-estimation/DebugTrace` for the trace build;
the Eclipse project keeps ST's name.)
The project finds the library through the path variable `CSP4CMSIS_ROOT`,
which points at `Lib/CSP4CMSIS` relative to the project; nothing else needs
configuring.

## Run (development boot)

Set both boot switches to the right (development mode, BOOT1 = 1) and power
the board. Then you can run from the IDE (Run or Debug button), or use the
scripts. These load the image into RAM through the ST-LINK GDB server and
run it; they write neither the external flash nor the OTP fuses (see
`Fuse_Programming()` below).

```bash
export STLINK_SN=<the DK's ST-LINK serial number>   # STM32_Programmer_CLI -l
export CUBEIDE=<STM32CubeIDE installation directory>
CSP/tools/run_dev.sh STM32CubeIDE/Debug/x-cube-n6-ai-multi-pose-estimation.elf 40
```

The console (VCP, 115200 8N1) shows ST's banner, then a stats line every 5 s:

    stats fps 30.30 inf 32ms cpu 38.4%

`fps` is 1000 divided by the period of the last frame (between the time
stamps isp gave two consecutive frames, in whole milliseconds), so it shows
single-frame values such as 29.41 (34 ms) and 30.30 (33 ms), not an average.
`inf` is that frame's inference time, `cpu` ST's CPU load over the last
second.

Only one board connected? Still set `STLINK_SN`: the scripts never guess a
probe. `VCP=<device>` overrides the serial port (default on Linux: the
ST-LINK's `/dev/serial/by-id` entry).

## Trace

```bash
export EVENTLIST=<path to CMSIS-Toolbox eventlist>
export PROBCLI=<path to probcli>        # optional: proofs
RING_POLICY=stop CSP_TOOLS=Lib/CSP4CMSIS \
  CSP/tools/trace_dev.sh STM32CubeIDE/DebugTrace/x-cube-n6-ai-multi-pose-estimation.elf 10
```

This loads and runs DebugTrace for 10 s, then interrupts it and dumps the
ring. `RING_POLICY=stop` keeps the first 16384 records from boot; without
it, the ring keeps the newest records. With `CSP_TOOLS` set, the script then
decodes the dump. It checks the trace against the CSP4CMSIS models
(`csp_trace.py`), with `CSP/tools/isr_writers.json` naming the vsync
interrupt as the one interrupt writer, and with `PROBCLI` set it proves the
checks. The output directory (`OUT`, default: a new temporary directory)
holds the dump, the decoded events, a Perfetto trace (`trace.json`, open it
locally in the Perfetto UI) and the conformance report (`conf/`).

## Results (STM32N6570-DK, 800 MHz, development boot)

Measured with this branch, built from a fresh clone as described above, each
configuration run for 60 s (11 stats lines):

- **Debug:** 29.41-30.30 fps (frame periods of 33-34 ms; the camera runs at
  30 fps), inference 32 ms, CPU load 38.3-38.4 %.
- **DebugTrace:** 28.57-31.25 fps (frame periods of 32-35 ms), inference
  32-33 ms, CPU load 38.2-39.9 %. Inference and CPU load are the same as in
  Debug; the single-frame periods spread more.
- **Conformance on silicon:** a DebugTrace capture from boot
  (`RING_POLICY=stop`: the first 16384 records, about 3.2 s) has 10
  projections: the 7 rendezvous channels, the concentrator's fair ALT and the
  2 KeepNewest channels. All trace checks (about 290 per capture; the count
  varies with timing) and the 6 stable-failure checks pass, proved with ProB.
  All 10 mutants (the trace with one injected fault) are caught. 85.7 % of
  the recorded CSP events lie in a checked projection.

## Fuse_Programming()

`Src/app_fuseprogramming.c` and the call in `main.c` are ST's code, unchanged.
At every boot, `Fuse_Programming()` reads OTP word 124 (`BSEC_HW_CONFIG_ID`).
If bits 15 and 16 (`HSLV_VDDIO3`, `HSLV_VDDIO2`) are not set, it programs
them, so that the xSPI pins can run at 200 MHz. **This is a one-time,
irreversible change to the chip.** ST's README states it ("OTP fuses are set
in this example").

The scripts in `CSP/tools/` put a breakpoint on `HAL_BSEC_OTP_Program`, the
HAL function that writes the fuses:

- If the bits are already set (for example on a board that has run ST's
  application), `Fuse_Programming()` returns without writing. The script
  prints `run_dev: Fuse_Programming returned without programming OTP`
  (`trace_dev: …` for `trace_dev.sh`) and runs the application.
- If the breakpoint is hit, the script prints
  `run_dev: HAL_BSEC_OTP_Program reached; resetting target, OTP not written`
  (`trace_dev: …`), resets the target, ends the debug session and exits with
  an error (`run_dev.sh` with status 2; `trace_dev.sh` prints
  `trace_dev: target not released` and exits with status 1).

So on a board whose bits are not yet set, the scripts will not run the
application. The IDE's Run and Debug buttons have no such guard: they run
ST's code, which programs the fuses. Whether to let ST's code program them,
for example by running ST's prebuilt binary once as ST's README describes,
is your decision.

## Licences

`CSP/` is this fork's code, MIT (`CSP/LICENSE`, Oliver Faust).
`Lib/CSP4CMSIS` is the CSP4CMSIS library, MIT (Oliver Faust). Everything
else is ST's or third-party code under the terms in `LICENSE.md` and in the
files themselves. This fork does not change those terms. The changes to ST's
files listed above are under ST's terms for those files.

---

<a id="sts-readme"></a>

# x-cube-n6-ai-multi-pose-estimation Application

Computer Vision application that run yolov8 pose estimation model on STM32N6570-DK board.

User should fetch yolov8 pose estimation model and deploy it for STM32N6570 NPU.

This top readme gives an overview of the app. Additional documentation is available in the [Doc](./Doc/) folder.

![Screenshot of application running](_htmresc/screenshot.png)

## Doc Folder Content

- [Download and deploy yolov8 pose estimation model](Doc/Download-and-deploy-model.md)
- [Application Overview](Doc/Application-Overview.md)
- [Boot Overview](Doc/Boot-Overview.md)
- [Camera build options](Doc/Build-Options.md)

## Features Demonstrated in This Example

- Multi-threaded application flow (FreeRTOS)
- Tracking and box/keypoints filtering (USER1 button enable/disable tracking/filtering feature)
- NPU accelerated quantized AI model inference
- Dual DCMIPP pipes
- DCMIPP crop, decimation, downscale
- LTDC dual-layer implementation
- DCMIPP ISP usage
- Dev mode
- Boot from External Flash

## Hardware Support

- MB1939 STM32N6570-DK
  - The board should be connected to the onboard ST-LINK debug adapter CN6 with a __USB-C to USB-C cable to ensure sufficient power__
  - OTP fuses are set in this example for xSPI IOs to get the maximum speed (200MHz) on xSPI interfaces.

- 4 Cameras are supported:
  - MB1854B IMX335 (Default Camera provided with the MB1939 STM32N6570-DK board)
  - [STEVAL-55G1MBI](https://www.st.com/en/evaluation-tools/steval-55g1mbi.html) VD55G1 Camera module (Use the CSI-2 cable provided with the camera module)
  - [STEVAL-66GYMAI1](https://www.st.com/en/evaluation-tools/steval-66gymai.html) VD66GY Camera module (Use the CSI-2 cable provided with the camera module)
  - [STEVAL-1943-MC1](https://www.st.com/en/evaluation-tools/steval-1943-mc1.html) VD1943 Camera module (Use the CSI-2 cable provided with the camera module)

![Board](_htmresc/ImageBoard.JPG)

STM32N6570-DK board with MB1854B IMX335.

## Tools Version

- IAR Embedded Workbench for Arm (**EWARM 9.40.1**) + N6 patch ([**EWARMv9_STM32N6xx_V1.0.0**](STM32Cube_FW_N6/Utilities/PC_Software/EWARMv9_STM32N6xx_V1.0.0.zip))
- [STM32CubeIDE](https://www.st.com/content/st_com/en/products/development-tools/software-development-tools/stm32-software-development-tools/stm32-ides/stm32cubeide.html) (**STM32CubeIDE 1.17.0**)
- [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html) (**v2.18.0**)
- [STEdgeAI](https://www.st.com/en/development-tools/stedgeai-core.html) (**v3.0.0**)

## Boot Modes

The STM32N6 does not have any internal flash. To retain your firmware after a reboot, you must program it in the external flash.
Alternatively, you can load your firmware directly from SRAM (dev mode). However, in dev mode, if you turn off the board, your program will be lost.

__Boot modes:__
- Dev mode (Both boot switches to the right): load firmware from debug session in RAM, program firmware in external flash
- Boot from flash (Both boot switches to the left)

## Console parameters

You can see application messages by attaching a console application to the ST-Link console output. Use the following console parameters:
- Baud rate of 115200 bps.
- No parity.
- One stop bit.

## Tracking

When tracking is enabled (enabled by default), post-processing is applied to
filters boxes and keypoints over frames to provide a more stable output. You can
disable/enable tracking at runtime by pressing the USER1 button on the board.

## Quickstart Using Prebuilt Binary

### Flash Prebuilt Binary

The binary must be programmed into the board's external flash using the following procedure:

  1. Set both switches to the right side.
  2. Press NRST button to reset the board.
  3. Program `Binary/x-cube-n6-ai-multi-pose-estimation.hex`.
  4. Set both switches to the left side.
  5. Press NRST to reset and start the application.
  6. The application will start and you should see the camera output. When
     pointing the camera to a person, you should see a skeleton-based
     representation with detected keypoints.

### How to Program Hex Files Using STM32CubeProgrammer UI

See [How to Program Hex Files STM32CubeProgrammer](Doc/Program-Hex-Files-STM32CubeProgrammer.md).

### How to Program Hex Files Using Command Line

Make sure to have the STM32CubeProgrammer bin folder added to your path.

```bash
export DKEL="<STM32CubeProgrammer_N6 Install Folder>/bin/ExternalLoader/MX66UW1G45G_STM32N6570-DK.stldr"

STM32_Programmer_CLI -c port=SWD mode=HOTPLUG -el $DKEL -hardRst -w Binary/x-cube-n6-ai-multi-pose-estimation.hex
```

## Quickstart Using Source Code

Before building and running the application, you have to [download and deploy](Doc/Download-and-deploy-model.md) yolov8
pose estimation model and then flash weights.

### Application Build and Run - Dev Mode

__Make sure to have both switches to the right side.__

#### STM32CubeIDE

Double click on `STM32CubeIDE/.project` to open the project in STM32CubeIDE. Build and run with the build and run buttons.

#### IAR EWARM

Double click on `EWARM/Project.eww` to open the project in IAR IDE. Build and run with the build and run buttons.

#### Makefile

Before running the commands below, be sure to have the commands in your PATH.

1. Build the project using the provided `Makefile`:

```bash
make -j8
```

2. Open a GDB server connected to the STM32 target:

```bash
ST-LINK_gdbserver -p 61234 -l 1 -d -s -cp <path-to-stm32cubeprogramer-bin-dir> -m 1 -g
```

3. In a separate terminal session, launch a GDB session to load the firmware image into the device memory:

```bash
$ arm-none-eabi-gdb build/Project.elf
(gdb) target remote :61234
(gdb) monitor reset
(gdb) load
(gdb) continue
```

### Application Build and Run - Boot from Flash

__Make sure to have both switches to the right side.__

#### STM32CubeIDE

Double click on `STM32CubeIDE/.project` to open project in STM32CubeIDE. Build with build button.

#### IAR EWARM

Double click on `EWARM/Project.eww` to open project in IAR IDE. Build with build button.

#### Makefile

Before running the commands below, be sure to have them in your PATH.

1. Build project using the provided `Makefile`:

```bash
make -j8
```

Once your app is built with Makefile, STM32CubeIDE, or EWARM, you must add a signature to the bin file:
```bash
STM32_SigningTool_CLI -bin build/Project.bin -nk -t ssbl -hv 2.3 -o build/Project_sign.bin
```

You can program the FSBL, the signed bin file at the address `0x70100000` and the network parameters:

```bash
export DKEL="<STM32CubeProgrammer_N6 Install Folder>/bin/ExternalLoader/MX66UW1G45G_STM32N6570-DK.stldr"

# First Stage Boot Loader
STM32_Programmer_CLI -c port=SWD mode=HOTPLUG -el $DKEL -hardRst -w FSBL/ai_fsbl.hex

# Network Parameters and Biases
STM32_Programmer_CLI -c port=SWD mode=HOTPLUG -el $DKEL -hardRst -w Model/network_data.hex

# Adapt build path to your IDE
STM32_Programmer_CLI -c port=SWD mode=HOTPLUG -el $DKEL -hardRst -w build/Project_sign.bin 0x70100000
```

Note: Only the App binary needs to be programmed if the FSBL and network_data.hex were previously programmed.

__Set both switches to the left side.__

Do a power cycle to boot from the external flash.

## How to update my project with a new version of ST Edge AI

The neural network model files (`network.c/h`, `stai_network.c/h`, etc.)
included in this project were generated using
[STEdgeAI](https://www.st.com/en/development-tools/stedgeai-core.html) version
3.0.0.

Using a different version of STEdgeAI to generate these model files may result
in the following compile-time error: `Possible mismatch in ll_aton library
used`.

If you encounter this error, please follow the STEdgeAI instructions on
[How to update my project with a new version of ST Edge AI Core](https://stedgeai-dc.st.com/assets/embedded-docs/stneuralart_faqs_update_version.html)
to update your project.

## Known Issues and Limitations
