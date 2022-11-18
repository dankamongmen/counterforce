# counterforce

[COUNTERFORCE on dankwiki](https://nick-black.com/dankwiki/index.php?title=Counterforce).

Counterforce is my project for next-level computer cooling, management, modeling, and visualization, borne out of a few years spent watercooling. My first nascent writeup was posted to reddit's r/watercooling on 2022-08-18. I want to bring Linux to the same levels of hardware awareness long available to Windows users through tools like HWiNFO...and then wildly surpass those levels. The ambitious end goal is to automatically model the physical cooling system, and associate it with sensors without human interaction. The rapid rise in power consumption and heat generation driven by recent CPUs, GPUs, and SSDs means more users concerned with what was once considered "extreme cooling" (AMD's second generation Threadripper processors, for instance, were explicitly marketed with an expectation of water cooling). I expect this project to require several years, along with some custom hardware, and it might prove infeasible for arbitrary builds.

The core principles of this effort include:

* Users ought not need to know (or pretend to know) the relationships between hardware control and cooling effect.
* Control of audible noise is almost as important as control of cooling effect.
* Changes within and without the system ought be integrated into control in realtime, and without user intervention.
* Quality visualization is critical for understanding the generation and elimination of heat.
* Closed source has no place in the coolant loop.

## the inaMORAta

[Obligatory dankwiki entry](https://nick-black.com/dankwiki/index.php?title=InaMORAta).

There are several controllers for the inaMORAta:
* `esp32`: Heltec WiFi LoRa 32 v2
* `esp32-wroom2`: HiLetGo ESP32
* `esp8266`: HiLetGo ESP8266
Look at `fanmgr.c` to get the mappings from pins to devices.

There is a MEGA2560 controller for the Geiger counter and sound detector in `mega2560/geiger`.
The MEGA2560 implementation of `fanmgr` is obsolete, and will be removed soon.

## resources

* Board definitions for Heltec ESP32 LoRa32 v2:
  * https://github.com/Heltec-Aaron-Lee/WiFi_Kit_series/releases/download/0.0.7/package_heltec_esp32_index.json

![simple 3970x die diagram](https://github.com/dankamongmen/counterforce/blob/master/doc/3970x.jpg?raw=true)
