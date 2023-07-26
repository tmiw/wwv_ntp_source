# wwv_ntp_source

This is a simple tool to decode WWV/WWVH signal to feed into NTP/Chrony. 

## Prerequisites

An audio source capable of 16 bit samples at 8 KHz sample rate (e.g. librtlsdr).

## Compiling the source code

```
git submodule update --init --recursive
mkdir build
cd build
cmake ..
make
``

## Running the application

Here's an example of how to use this with librtlsdr:

```
sudo ../librtlsdr/build/src/rtl_fm -f 10000000 -M am -s 8k -E dc -E direct2 - | ./wwv
```

The above execution of `rtl_fm` tunes a RTL-SDR or similar to 10 MHz AM (one of the 
frequencies used by WWV/WWVH) and feeds the audio to this tool at 8 KHz sample rate.

### Remaining work

* Feed time data into NTP/Chrony via SHM interface.

### License

See [LICENSE.md](./LICENSE.md) for more details.


