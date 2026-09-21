# rtl_tcp_soapy

### Basically vibe-coded: Written almost entirely by DeepSeek V4.1 Flash, but it works well and was reviewed extensively.

`rtl_tcp_soapy` is an I/Q spectrum server. It uses the network protocol of
`rtl_tcp`. It controls a SoapySDR device instead of an RTL-SDR dongle.

Many programs support the `rtl_tcp` protocol. This server lets those programs
use other radios, for example a USRP, a LimeSDR, or a PlutoSDR. The server
converts the samples to the unsigned 8-bit format that the clients expect.

## Contents

- [How it works](#how-it-works)
- [Protocol](#protocol)
- [Requirements](#requirements)
- [Build](#build)
- [Usage](#usage)
- [Examples](#examples)
- [Behavior notes](#behavior-notes)
- [License](#license)

## How it works

The server does these steps:

1. It opens a SoapySDR device.
2. It sets the sample rate, the center frequency, and the gain.
3. It listens on a TCP port.
4. When a client connects, it sends a 12-byte handshake.
5. It streams I/Q samples to the client.
6. It applies each command that the client sends.

The server serves one client at a time. This is the same behavior as
`rtl_tcp`.

## Protocol

### Handshake

The server sends 12 bytes when a client connects.

| Bytes   | Value                              |
|---------|------------------------------------|
| 0 to 3  | The ASCII text `RTL0`              |
| 4 to 7  | The tuner type, big-endian         |
| 8 to 11 | The number of gain values, big-endian |

### Sample stream

The server sends I/Q samples continuously. Each sample has two bytes. The first
byte is I. The second byte is Q. The values are unsigned. The value 128 is
zero.

### Commands

A command packet has 5 bytes. Byte 0 is the command. Bytes 1 to 4 are a
big-endian value.

| Command | Action                                          |
|---------|-------------------------------------------------|
| 0x01    | Set the center frequency in Hz                  |
| 0x02    | Set the sample rate in Hz                       |
| 0x03    | Set the gain mode (0 = automatic, 1 = manual)   |
| 0x04    | Set the gain in tenths of a dB                  |
| 0x05    | Set the frequency correction in ppm             |
| 0x06    | Set the IF stage gain                           |
| 0x07    | Set the test mode (ignored)                     |
| 0x08    | Set the AGC mode                                |
| 0x09    | Set the direct sampling mode (ignored)          |
| 0x0a    | Set the offset tuning mode (ignored)            |
| 0x0b    | Set the RTL crystal frequency (ignored)         |
| 0x0c    | Set the tuner crystal frequency (ignored)       |
| 0x0d    | Set the gain by index                           |
| 0x0e    | Set the bias tee                                |

## Requirements

- A C compiler
- CMake 3.7 or later
- SoapySDR and the driver module for your radio

## Build

Do these steps:

    cmake -S . -B build
    cmake --build build

The program is at `build/rtl_tcp_soapy`.

To install the program and the manual page, run this command:

    cmake --install build

## Usage

Run the program with a SoapySDR device argument and a listen address:

    rtl_tcp_soapy --args "driver=uhd" --listen 127.0.0.1:1234

### Device options

| Option                  | Description                                                                 |
|-------------------------|-----------------------------------------------------------------------------|
| `--args <args>`         | SoapySDR device arguments. Default: the first device found.                 |
| `-c, --channel <n>`     | The RX channel index. Default: 0.                                           |
| `-f, --freq <Hz>`       | The initial center frequency. Default: 100000000.                           |
| `-s, --samplerate <Hz>` | The initial sample rate. Default: 2048000.                                  |
| `-b, --bandwidth <Hz>`  | The RF filter bandwidth. Default: the driver default.                       |
| `-g, --gain <dB>`       | The initial gain. This option enables manual gain mode. Default: automatic gain control. |
| `-T, --bias-tee`        | Enable the bias tee at startup.                                             |

You can use a suffix for a frequency value. For example, `100M` is 100000000.

### Server options

| Option              | Description                                                |
|---------------------|------------------------------------------------------------|
| `--listen <addr>`   | The listen address. Use `host:port`, `:port`, or `host`.   |
| `-a, --addr <addr>` | The listen address. Default: 127.0.0.1.                    |
| `-p, --port <port>` | The listen port. Default: 1234.                            |

### Compatibility options

| Option                  | Description                                                            |
|-------------------------|------------------------------------------------------------------------|
| `--tuner-type <n>`      | The RTL-SDR tuner type in the handshake. Default: 5 (R820T).           |
| `-D, --direct-sampling` | Accepted for compatibility. The server ignores this option.            |
| `-h, --help`            | Show the help text.                                                    |

## Examples

Serve a USRP on the default rtl_tcp port:

    rtl_tcp_soapy --args "driver=uhd" --listen 127.0.0.1:1234

Serve a LimeSDR on all interfaces:

    rtl_tcp_soapy --args "driver=lime" --listen 0.0.0.0:1234 -f 100M -s 2.048M

Serve a PlutoSDR that runs the Tezuka firmware:

    rtl_tcp_soapy --args "driver=tezuka,hostname=192.168.3.1" --listen 127.0.0.1:6969

In the client program, use the device string `rtl_tcp=<host>:<port>`. For
example, use `rtl_tcp=127.0.0.1:1234`.

## Behavior notes

- The server serves one client at a time. This is the same behavior as
  `rtl_tcp`.
- The server applies the command-line configuration again for each new client.
  So a client always starts from a known state. This is important for clients
  that do not set the sample rate and that expect the `rtl_tcp` default of
  2048000 Hz.
- The server reads the gain range of the device at startup. It builds a gain
  table with 29 steps across this range. The highest gain index sets the
  highest gain of the device.
- The server reports the R820T tuner type (5) in the handshake. It reports 29
  gain values.
- The server accepts commands for RTL-SDR hardware features, but it ignores
  them. These commands are direct sampling, offset tuning, test mode, and the
  crystal frequencies.
- The bias tee command uses the `biastee` and `bias_tee` settings of the
  device. Some devices do not support these settings.

## License

This program is free software. You can redistribute it and/or modify it under
the terms of the GNU General Public License, version 2 or later. See the file
`COPYING` for the full text.
