# wwv_ntp_source

This is a simple tool to decode WWV/WWVH signal to feed into NTP/Chrony. 

## Prerequisites

An audio source capable of 16 bit samples at 8 KHz sample rate (e.g. librtlsdr).

## Compiling the source code

```
$ git submodule update --init --recursive
$ mkdir build
$ cd build
$ cmake ..
$ make
```

## Running the application

Here's an example of how to use this with librtlsdr:

```
$ pwd
/home/user/wwv_ntp_source/build
$ sudo ../../librtlsdr/build/src/rtl_fm -f 10000000 -M am -s 8k -E dc -E direct2 - | ./src/wwv
Found 1 device(s):
  0:  Realtek, RTL2838UHIDIR, SN: 00000001

Using device 0: Generic RTL2832U OEM
Detached kernel driver
Found Rafael Micro R820T tuner
Tuner gain set to automatic.
Enabled direct sampling mode, input 2
Enabled direct sampling mode, input 2/Q.
Tuned to 10252000 Hz.
Oversampling input by: 126x.
Oversampling output by: 1x.
Buffer size: 8.13ms
Exact sample rate is: 1008000.009613 Hz
Sampling at 1008000 S/s.
Output at 8000 Hz.
Locked onto WWV signal

R01011000P101000010P010000000P111000000P010000000P101001000
Date: Day 207 of year 2023
Time (UTC): 2:45

...
```

The above execution of `rtl_fm` tunes a RTL-SDR or similar to 10 MHz AM (one of the 
frequencies used by WWV/WWVH) and feeds the audio to this tool at 8 KHz sample rate.

### Remaining work

* Feed time data into NTP/Chrony via SHM interface.

### License

See [LICENSE](./LICENSE) for more details.


