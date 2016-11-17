## Overview

This app is a ble peripheral app which advertises so that the central can connect to it. Currently it does not readvertise after a single connection is established. It is a copy of Apache Mynewt's bleprph app with some modifications.

## Modifications to bleslave app ##

- Changing the name to bleslave to avoid confusion in functionality
- Changing dependencies to point to Apache mynewt core repo
- Using configurable LED pin if someone wants to do a custom LED hookup
- Toggle LED when it connects and turns off the LED when it disconnects
- Uses configurable last byte for the address
- Using os_callout for running a timer to toggle the LED

## Functionality ##

This app performs the following functions:

- Advertises the name "runtime-XX" where XX is configured from the
  target using the syscfg variable
- Stops advertising once connected
- Starts blinking LED on connecting
- Is the ble central disconnects the LED stops blinking
