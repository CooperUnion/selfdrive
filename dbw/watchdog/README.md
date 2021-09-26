# IGVC DBW Team's Watchdog

![Build Action](https://github.com/autonomy-lab-cooper-union/dbw_watchdog/actions/workflows/c-cpp.yml/badge.svg)

The DBW watchdog is a program that runs on the Pi, along with the UI software. It 
is designed to monitor the rest of the car in a central place and be the 
authority for mode changes.

The watchdog is composed of a core and main thread, which starts module threads that monitor various aspects
of the car.

### Dev:
Please branch and PR; name branches roughly speaking according to the area being changed -

- core_xxx -> changes to the core
- watchdog_xxx -> changes to the watchdog
- mod_canhealth_xxx -> changes to mod_canhealth, etc.

Remember, build passing here doesn't necessarily mean build passing on ARM/the Pi, so please check both before merging.
