#!/bin/bash

avrdude -pm328p -cdragon_pp -D -Ueeprom:w:Release/avr_DF4IAH_10MHz_Reference.eep:a -Uflash:w:Release/avr_DF4IAH_10MHz_Reference.hex:a

echo "==="
echo "Please allow following verification error:  > first mismatch at byte 0x78f8    0x04 != 0xa5 <" 
echo

