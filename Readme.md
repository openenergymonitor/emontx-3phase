# emonTx V3 3-phase Firmware

[![Build Status](https://travis-ci.org/openenergymonitor/emontx-3phase.svg?branch=master)](https://travis-ci.org/openenergymonitor/emontx-3phase)

This sketch is a development of the 3-phase ‘discrete sample’ sketch and MartinR’s PLL
energy diverter.

It is intended for use on a **3-phase, 4-wire system**. It utilises advanced
features of the Atmel328P microprocessor to provide continuous monitoring of voltage of
one phase and the currents in all three phases, thereby allowing a good estimate of real
power to be calculated. The physical quantities are measured continuously, the average
values are calculated and transmitted at user-defined intervals.

Pulse counting and temperature monitoring using a single DS18B20 sensor is supported.

*By [Robert Wall](https://community.openenergymonitor.org/u/robert.wall/summary)*

## Key Feautures

- Continuous monitoring of one voltage channel and up 4 current channels.
- Gives an accurate measure of rapidly varying loads.
- 1800 sample sets per second – using 4 channels of an emonTx V3.4 @ 50 Hz
- Calculates rms voltage, rms current, real & apparent power & power factor.
- Pulse input for supply meter monitoring.
- Integrated temperature measurement (one DS18B20 sensor).
- User-defined reporting interval.
- Suitable for operation on a three-phase, 4-wire supply at 50 or 60 Hz.
- Can be calibrated for any voltage and current (default calibration is for emonTx with 100 A CTs & UK a.c. adapter).

## Limitations

Because the voltage of only one phase can be measured, the sketch must assume that
the voltages of the other two phases are the same. This will, in most cases, not be true,
therefore the powers calculated and recorded will be inaccurate. However, this error
should normally be limited to a few percent. Voltage imbalance might be the result of
unbalanced loads within your installation, or it might result from the actions of other
consumers on the same supply.

If you find that one or other phase voltage is consistently wrong, you might want to
consider a small adjustment to the current calibration of the affected phase.
Because of the additional power required for continuous operation, a 5 V USB power
supply is likely to be required, even if only the on-board RFM69CW radio is used. The 5 V
power will always be required when the ESP8266 WiFi module is added.

The sketch is not compatible with the RFM12B radio module, nor the Arduino Due.


***

## Resources

### [Introduction to three-phase](https://learn.openenergymonitor.org/electricity-monitoring/ac-power-theory/3-phase-power)

### [Full 3-phase Firmware User Guide](emontx-3-phase-userguide.pdf)
*Including calibration instructions*

## Required Hardware

- emonTx V3.4
- 3 x Clip on CT sensors
- 1 x AC-AC adapter
- USB to UART programmer (to upload firmware)

## Upload pre compiled

- Firmware can be uploaded directly using [emonUpload](https://github.com/openenergymonitor/emonupload) tool to grab latest compiled release and upload via serial.

or

- Download pre-compiled `.hex` from [github releases](https://github.com/openenergymonitor/emontx-3phase/releases) and upload using avrdude

Depending on your ISP

`avrdude  -uV -c arduino -p ATMEGA328P -P /dev/ttyUSB0 -b 115200 -U flash:w:firmware.hex`


## Compile & Upload

- Firmware can be compiled and uploaded with PlatformIO, see [User Guide](https://guide.openenergymonitor.org/technical/compiling)

Or

- Arduino IDE


Changed by DB.. 
. Signature checking. Problem: First time use wouldn’t give any usable results as the original sketch would check at EEPROM address for byte 255, which would be the case only if someone had gone through the calibration routine or done ‘wipe EEPROM’ in ‘+++’ config.
	Solution: instead I asked the sketch to check at EEPROM address 999 for the byte 035 (035 was my spelling of OEM :) )
			if the signature is not found then sketch defaults are used.
			- I’ve tested saving to EEPROM calibration settings, this writes the signature.
			- I’ve tested wiping the EEPROM, this removes the signature instead of wiping all values up to address ‘255'.

. Wh_accumulator on device for each CT channel. Needless to say, this is useful. Wh are sent over ESP, RFM or SERIALOUT as selected.
. PowerFactor information sent with ESP, RFM or SERIALOUT as selected.
. #define CT4Phase selection comments out or in with more logical consistency throughout sketch.
. #define TEMPERATURE_SENSING FOR using the temp sensors or not, I didn’t have mine soldered on the Tx and thus didn’t want that information six times over as the sketch default was.
. #define USEPULSECOUNT disables sending pulse counting information more consistently across  ESP, RFM or SERIALOUT as selected.
