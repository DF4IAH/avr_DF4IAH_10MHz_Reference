#!/bin/sh

avrdude -cusbasp-clone -pm328p -D -Ulfuse:v:0xbf:m -Uhfuse:v:0xd8:m -Uefuse:v:0xfd:m -Uee:w:DF4IAH_10MHz_Reference_Appl.eep -Ufl:w:DF4IAH_10MHz_Reference_Appl.hex

