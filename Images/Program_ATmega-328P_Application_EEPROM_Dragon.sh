#!/bin/bash

avrdude -pm328p -cdragon_pp -D -Ueeprom:w:../DF4IAH_10MHz_Reference_Appl/DF4IAH_10MHz_Reference_Appl/Release/DF4IAH_10MHz_Reference_Appl.eep:a

echo "==="
echo "Programming done." 
echo

