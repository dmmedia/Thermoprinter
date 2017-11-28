# Thermoprinter
Thermoprinter firmware

University project to create a portable, bluetooth enabled thermoprinter, that is able to communicate different protocols.

Program structure is ported from [Marlin](https://github.com/MarlinFirmware/) and reduced to a single axis.
Endstops are changed to other external interrupts.
Added battery voltage measurement in addition to the temperature.

Simple protocol is to be implemented for testing purposes:
* M0 [-]X : move paper X steps [backward]
* P0 48_BYTE_HEX_STRING : print 48 bytes = 384 bits line to the paper and advance 2 steps

20 steps backward and 20 steps forward will be inserted before the first P0 command and 20 steps forward after last P0 command, as required per MLT-288 print head datasheet.
