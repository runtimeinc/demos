## Overview

This app is a ble central app which connects to 32 peripherals
simultaneously. It is a copy of Apache Mynewt's blecent app with some
modifications.

## App differences: blemaster app Vs blecent app ##

- Changing the name to blemaster to avoid confusion in functionality
- Changing dependencies to point to Apache mynewt core repo
- Address list to allow connection to specific peers
- Using specific connection parameters
- UART configuration and buffers and driver initialization for talking to Nextion displays, also removing blecent specific APIs(Commented out for now, will be included once everything works)
- BLE Scan can fail if it is already scanning, hence we do not want to look at the return code
- Using configurable LED pin if someone wants to do a custom LED hookup
- Adding syscfg to allow for more number of connections and reducing the log level

## Functionality ##

This app performs the following functions:
- Keeps scanning continously. So, it can reconnect to any devices that
  get disconnected and start readvertising.
- Does a lookup for specific peers once advertisements are heard by the
  master and only connects to ones that exist in the peerlist(This is
  not using the whitelist feature of the controller)
- Turns on the LED at startup. The LED can be configured using syscfg.

