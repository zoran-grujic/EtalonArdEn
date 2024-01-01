# Arduino code for Etalon

Implementation of a lock-in amplifier for detection of 3-rd harmonic at 1.3 kHz.

Use Arduino IDE to edit, compile and upload. 

More details at https://github.com/zoran-grujic/EtalonPy

## Compatibility

Only Arduino DUE.

# Commands

scan 2345 2345
---------------------------
Set HV PZT over SPI 16 bit dac to be 2345 * 4,096/(2&16)

Responce: 
Error reporting: report error if numbers are not equal


whois? 
-------------------------
Responce: Etalon lock-in

mode?
---------------------------
Responce: mode: scan or mode: lock-in

phase 9001 9001
----------------------
Postavlja fazu na 90,01 deg.

Responce: phase 9001
Error reporting: report error if numbers are not equal

moveHV 60 60
---------------------------
Increase OC position by 60 

Responce: none
Error reporting: report error if numbers are not equal

lock 32768 32768
-------------------------
Try to lock at 4,096/2 V HV PZT 

Responce: mode: lock
Responce: Lok point: 32768
Error reporting: report error if numbers are not equal

PI 587 654
-------------------------
Postavlja integralni (587) i proporcionalni deo (654)

Responce: none

lockpoint 54 54 
----------------------------

Responce: Lock point offset: 54
Error reporting: report error if numbers are not equal

center
-----------------------------------
call center finction if mode is lock

FHVratio 1000 1000
-----------------------------------
Set HR and OC PZT ratio

Error reporting: report error if numbers are not equal


## GIT

Arduino Due code is here: https://github.com/zoran-grujic/EtalonArdEn.git
GUI interface is here: https://github.com/zoran-grujic/EtalonPy.git
