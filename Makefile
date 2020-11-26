##
## This file is part of the libopencm3 project.
##
## Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
##
## This library is free software: you can redistribute it and/or modify
## it under the terms of the GNU Lesser General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## This library is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU Lesser General Public License for more details.
##
## You should have received a copy of the GNU Lesser General Public License
## along with this library.  If not, see <http://www.gnu.org/licenses/>.
##

PROJECT = feather
BINARY = feather
BUILD_DIR = bin

DFU_ADDRESS = 0x08002000
#LDSCRIPT = feather.ld 

# append currdir to includes?
INCLUDE=-I$(CURDIR)
CFLAGS+= $(INCLUDE)

# Project Source Files
CFILES+=feather.c
CFILES+=afsk.c
CFILES+=si446x.c
CFILES+=delay.c
CFILES+=ax25.c
CFILES+=aprs.c
CFILES+=gps.c
CFILES+=usart.c
CFILES+=sleep.c
CFILES+=adc.c

# TODO - you will need to edit these two lines!
DEVICE=stm32f031g6u6
OOCD_FILE = board/stm32f0discovery.cfg

# You shouldn't have to edit anything below here.
VPATH += $(SHARED_DIR)
INCLUDES += $(patsubst %,-I%, . $(SHARED_DIR))
OPENCM3_DIR=./libopencm3

include $(OPENCM3_DIR)/mk/genlink-config.mk
include ./rules.mk
include $(OPENCM3_DIR)/mk/genlink-rules.mk

