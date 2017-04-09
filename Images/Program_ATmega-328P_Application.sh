#!/bin/bash

avrdude -pm328p -cdragon_pp -D -Ueeprom:w:../Release/avr_DF4IAH_10MHz_Reference.eep:a -Uflash:w:../Release/avr_DF4IAH_10MHz_Reference.hex:a
# avrdude -pm328p -cusbasp-clone -D -Ueeprom:w:../Release/avr_DF4IAH_10MHz_Reference.eep:a -Uflash:w:../Release/avr_DF4IAH_10MHz_Reference.hex:a

echo "==="
echo "Please allow following verification error:  > first mismatch at byte 0x7780    0xff != 0xe4 <" 
echo

