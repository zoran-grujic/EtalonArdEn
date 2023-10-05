# Arduino code for Etalon

Implementation of a lock-in amplifier for detection of 3-rd harmonic at 1.3 kHz.

Use Arduino IDE to edit, compile and upload. 

More details at https://gitlab.com/zoran-grujic/etalonpy

## Compatibility

Only Arduino DUE.

# Komande

whois? 
-------------------------
Odgovor: Etalon lock-in

mode?
---------------------------
Odgovor: mode: scan ili mode: lock-in

scan 2345 2345
---------------------------
Postavlja HV PZT preko SPI 16 bit dac na vrednost 2345 * 4,096/(2&16)

Odgovor: ako dva broja nisu ista prijavljuje grešku

phase 9001 9001
----------------------
Postavlja fazu na 90,01 deg.

Odgovor: ako dva broja nisu ista prijavljuje grešku

lock 32768 32768
-------------------------
Pokušava da se lokuje na 4,096/2 V HV PZT-a. 

Odgovor: mode: lock

Odgovor: Lok point: 32768

PI 587 654
-------------------------
Postavlja integralni (587) i proporcionalni deo (654)

FHVratio 1000 1000
-----------------------------------
Postavlja odnos HV i običnog PZT-ako

Odgovor: Ako brojevi nisu isti prijavljuje grešku





