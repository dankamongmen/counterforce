# counterforce

this program runs on a Libre Le Potato in my workstation (the Potato is
a knockoff of a Raspberry Pi 3B). said SBC is hooked up via HDMI and USB
to a [Waveshare 5.5" AMOLED](https://nick-black.com/dankwiki/index.php?title=Waveshare_AMOLED)
capacitive screen having 1920x1080 resolution.

it receives data from a program running on the workstation proper, and
draws to the AMOLED. it is necessary that it run in some manner of
attract mode, as the AMOLED is susceptible to burnin. we currently run
xscreensaver, but ideally we would not.

## resources

* Board definitions for Heltec ESP32 LoRa32 v2:
  * https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
  * https://github.com/Heltec-Aaron-Lee/WiFi_Kit_series/releases/download/0.0.7/package_heltec_esp32_index.json

## UI

right now only a monitoring screen and a plot screen are available. later,
there will probably be more screens you can flip through.

at all times, we ought show a green or red indication based on whether we've
received data in the past 2s; display the time since last receipt at all times.

### monitoring screen

separated into three panes:

* rendition of motherboard side of machine
  * shows fans, radiators, hard drives, m2s, pumps, reservoir
  * temp sensor follows Quantum Kinetic FLT
  * flow data from aquacomputer NEXT sensor
  * show ambient temp, coolant temp, flow, and delta-T
* rendition of PSU side of machine
  * shows fans, radiators, hard drives
  * temp sensor follows bottom radiator
  * UPS info (power lost? connection status, battery status)
  * power consumption as reported by UPS
* rendition of 3970X and TU104
  * 3970X is a quincunx of 4 CCDs and central I/O die
  * show power draw of both (`nvidia-smi`, `turbostat`)
  * 4 temperature sensors are mapped arbitrarily to CCD representations
  * show busiest processes on all execution units

![simple 3970x die diagram](https://github.com/dankamongmen/counterforce/blob/master/doc/3970x.jpg?raw=true)

whenever possible, sensor results are displayed atop relevant sensing
point. think damage/HP in final fantasy iii.

fans ought show RPMs and direction of airflow. the RPM count ought
intensify as it increases.

it would be cool to be able to press a bit of hardware and have an actual image
of it show up along with details, until you dismiss this new bit of display.

* would be nice to sense pressure both in coolant and in case
* would be nice to display volume of noise being generated by machine
  * ideally we'd not reflect ambient noise (how?)

### plot screen

plots use 1s buckets. we probably ought add a capability to see hours, days, etc., but
for now it's all run off seconds. data thus oughtn't be expensive to collect.

* 64-way line plot of process count on each cpu
* temps for each CCD and GPU, ambient, and coolant
* 16 + 5-way line plot of I/O on each block device
* network load on each interface (probably each has its own graph?)
