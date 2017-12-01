#!/bin/bash

#avrdude -pm328p -cdragon_pp -D -Uflash:w:../DF4IAH_10MHz_Reference_Appl/DF4IAH_10MHz_Reference_Appl/Release/DF4IAH_10MHz_Reference_Appl.hex:a
avrdude -pm328p -cusbasp-clone -D -Uflash:w:../DF4IAH_10MHz_Reference_Appl/DF4IAH_10MHz_Reference_Appl/Release/DF4IAH_10MHz_Reference_Appl.hex:a

echo "==="
echo "Programming done." 
echo

