# RA8P1 Titan Mini

Initial CPU0 BSP for the RA8P1 Titan Mini (R7KA8P1KF, Cortex-M85).
The port uses native NuttX startup and lower-half drivers. No Renesas FSP or
RT-Thread source is built or vendored by this port.

## Supported configuration

- CPU0 secure flat build, 24 MHz crystal, CPU clock 1 GHz.
- First 1 MiB of MRAM at `0x02000000` for code.
- First 448 KiB of SRAM at `0x22000000` for data, stacks and heap.
- SCI2 console and SCI1 secondary UART at 115200 baud, 8N1.
- RMAC1/RGMII Ethernet, RTL8211F PHY, IPv4, ARP, ICMP, UDP and TCP.
- DHCP and DNS, with `ifconfig`, `ping` and `renew` in NSH.
- 32 MiB external SDRAM, with the low 2 MiB reserved for camera DMA.
- IIC0 and IIC1 NuttX I2C masters.
- QSPI MTD, SDHI/MMC-SD, SPI1 and CAN FD lower-halves.
- USBFS device controller with CDC ACM.
- SSI0 I2S playback, ADC, RTC, WDT, GPT PWM and GPIO devices.
- OV5640 MIPI CSI-2/VIN video capture and stereo PDM capture.
- Arm Ethos-U55 AI engine with Vela command-stream invocation.
- Instruction cache controlled by `CONFIG_ARMV8M_ICACHE`.

CPU1 and protected/kernel builds are not supported. Ethernet, camera and
Ethos-U currently require D-cache off; Kconfig enforces those constraints.
DMA-visible storage is placed in internal SRAM or the reserved SDRAM camera
region.

## Device interfaces

The `titan-mini:nsh` configuration enables the board peripherals and test
commands. Depending on attached hardware, it provides:

| Function | NuttX interface |
|---|---|
| UART | `/dev/console` and `/dev/ttyS0` on SCI2, `/dev/ttyS1` on SCI1 |
| Ethernet | `eth0` |
| I2C | `/dev/i2c0`, `/dev/i2c1` |
| OSPI flash | `/dev/ospi0`, `/dev/ospi_fs`, `/fal` |
| microSD | `/dev/mmcsd0` when a card is inserted |
| SPI | `/dev/spi1` |
| CAN FD | `/dev/can0` |
| USB CDC ACM | `/dev/ttyACM0` |
| PCM playback | `/dev/audio/pcm0` |
| PDM capture | `/dev/pdm0` |
| Camera | `/dev/video` |
| ADC / RTC / WDT | `/dev/adc0`, `/dev/rtc0`, `/dev/watchdog0` |
| LEDs / button / GPIO / PWM | `/dev/userleds`, `/dev/buttons`, `/dev/gpio*`, `/dev/pwm*` |
| Ethos-U55 | `/dev/ethosu0` |

## Console and network wiring

Route SCI2 to PMOD1 through SW4:

| Signal | Board pin | Connection |
|---|---|---|
| SCI2 TX | P8.01 | USB UART RX |
| SCI2 RX | P8.02 | USB UART TX |
| GND | GND | USB UART GND |

Use a 3.3 V TTL adapter. SCI1 remains available as `/dev/ttyS1` but is not
the default console. Connect the board RJ45 to a DHCP-enabled Ethernet network.
The board data path uses RMAC1, but PHY management is through RMAC0 MDIO; both
ETHA ports must be initialized. The PHY address is 1. The default locally
administered MAC is derived from the MCU unique ID.

The hardware validation setup connects the RJ45 directly to `box`:
`box` provides `192.168.77.1/24`, DHCP, DNS forwarding and NAT over Wi-Fi
using the existing `titan-ethernet-router.service`.

## Build

Use Arm GNU Toolchain **13.3.Rel1** (`arm-none-eabi-gcc`), GNU Make,
Kconfig tools and a sibling `apps` checkout. On macOS, the apps build also
requires `flock`. Put the toolchain and Kconfig executables on `PATH`.

From the NuttX directory:

```sh
./tools/configure.sh -m titan-mini:netnsh
make -j10
```

Use `-l` instead of `-m` for a Linux host. `titan-mini:nsh` is the smaller
serial-only configuration. Check for another build in this same checkout
before changing configuration or starting a build.

The output is `nuttx` (ELF) and `nuttx.hex`. The linker script does not
program option-setting memory, external memory, or CPU1 memory.

## Flash and validate

The verified probe is HSLink Pro in CMSIS-DAP mode. Install pyOCD and
Renesas.RA_DFP 6.6.0 to make target `R7KA8P1KF` available:

```sh
python3 -m pyocd load -t R7KA8P1KF --format elf nuttx
python3 -m pyocd commander -t R7KA8P1KF -c reset -c continue
```

Select the probe with `-u SERIAL` if more than one is connected. The DFP's
SVD parser can emit an exception in the tested pyOCD version; check actual
program/verify completion and the serial boot log. Do not run two serial
readers or two SWD operations concurrently.

After boot, NSH starts while the network initialization thread negotiates
the link and obtains a lease:

```text
nsh> uname -a
nsh> ifconfig
nsh> ping -c 5 192.168.77.1
nsh> ping -c 5 223.5.5.5
nsh> ping -c 5 www.baidu.com
nsh> ping -c 5 -s 1472 192.168.77.1
nsh> free
```

The gateway address is specific to the validation setup. Check the address,
mask and default router printed by `ifconfig` on other networks.

The driver polls link state every 500 ms and retains the DMA transmit
buffer until completion. Only one packet is transmitted at a time in this
initial implementation. Receive storage is eight 1536-byte buffers plus
two pairs of descriptor queues. TX timeout restarts the device.

DHCP initialization is a one-shot NSH startup operation. If the cable was
absent during startup or the network changes, reconnect it and run
`renew eth0`. To exercise interface restart:

```text
nsh> ifdown eth0
nsh> ifup eth0
nsh> renew eth0
```

## Driver model

RA8P1 peripheral drivers are native NuttX lower-halves under
`arch/arm/src/ra8p1`. Board files contain pin assignments, external-device
configuration and registration only. Each optional peripheral is controlled
by Kconfig and is omitted from the build when disabled.

The Arm Ethos-U core driver under `drivers/aie/ethosu` is an upstream
Apache-2.0 component. RA8P1 power, clock, interrupt, memory validation and
the NuttX AI engine interface remain in the architecture lower-half.
