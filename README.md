# ExpressLRS Multi-RX Tester

Using an ESP32 devkit, a micro-sd module, a buck converter and an RGB LED we can collect receiver statistics from 6
receivers at once to provide a comparison of receiver sensitivity and performance on an even playing field.

The first 2 ports are hardware serial and the remaining 4 are emulated serial receivers
using the built-in RMT device.

![Top View](images/top.png)
![Bottom View](images/bottom.png)

# Connections

| ESP32  | Connection       |
|--------|------------------|
| GPIO14 | Port 1 TX pin    |
| GPIO27 | Port 2 TX pin    |
| GPIO26 | Port 3 TX pin    |
| GPIO25 | Port 4 TX pin    |
| GPIO33 | Port 5 TX pin    |
| GPIO32 | Port 6 TX pin    |
| GPIO13 | RGB LED DI pin   |
| GPIO5  | SD Card SS pin   |
| GPIO18 | SD Card SCK pin  |
| GPIO19 | SD Card MISO pin |
| GPIO23 | SD Card MOSI pin |

# Usage

Format the SD card as FAT32, after plugging in the power the LED will show a blue color indicating it is awaiting start.
Press the Boot button on the ESP32 dev board and the recording will start and the LED will turn green.
If there is a problem opening the SD card, the LED will flash red and return to the blue color. Press the Boot button again to close the log before removing power.  The LED will show blue again.

A log of information is written to the SD card as `/log000.csv`, each line in the CSV is a line read from a receiver prefixed
with the port number (1-6).

Receivers under test should be compiled with the `-DRCVR_UART_BAUD=460800` flag and either of `-DDEBUG_RCVR_SIGNAL_STATS` or 
`-DDEBUG_RCVR_LINKSTATS` flags, and all but one receiver should have telemetry disabled in the web UI using the
"Force telemetry OFF on this receiver" option, as shown below.

![Telemetry off](images/Telemetry%20off.png)

There is debugging/operation information printed on the main USB connector at 921600 baud.

