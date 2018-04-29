#/*
# * Copyright (c) 2016 Rosimildo DaSilva <rosimildo@gmail.com>
# *
# * This program is free software; you can redistribute it and/or modify
# * it under the terms of the GNU General Public License as published by
# * the Free Software Foundation; either version 2 of the License, or
# * (at your option) any later version.
# *
# * This program is distributed in the hope that it will be useful,
# * but WITHOUT ANY WARRANTY; without even the implied warranty of
# * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# * GNU General Public License for more details.
# *
# * You should have received a copy of the GNU General Public License
# * along with this program; if not, write to the Free Software
# * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
# * MA 02110-1301, USA.
# *
# */

CC = gcc
CPP =g++
STRIP = strip
AR = ar

TARGET=rtmp_camera
BUILDPATH=.

SRCDIRS:=. Camera watermark

CFLAGS =-Wall -O3 -ldl -pthread -std=c++11

INCLUDES:=$(foreach dir,$(SRCDIRS),-I$(dir)) -I. -I./libyuv/include

SRCCS=$(foreach dir,$(SRCDIRS),$(wildcard $(dir)/*.c))
SRCPPS=$(foreach dir,$(SRCDIRS),$(wildcard $(dir)/*.cpp))

LIBOBJ=$(addprefix $(BUILDPATH)/, $(addsuffix .o, $(basename $(SRCCS))))
LIBOBJ+=$(addprefix $(BUILDPATH)/, $(addsuffix .o, $(basename $(SRCPPS))))

LDFLAGS= -ldl -lm -lpthread ./libyuv/libyuv.a ./librtmp/librtmp.a -lpthread -lssl  -lrt -g -lcrypto -lz
ifeq ($(shell pkg-config --exists cedarx; echo $$?),0)
LDFLAGS += $(shell pkg-config --libs cedarx)
else
LDFLAGS += -lvencoder -lcdx_base -lMemAdapter -lVE -L/usr/local/lib/cedarx
endif

all: $(TARGET)

$(BUILDPATH)/%.o:%.c
	$(CC) $(CFLAGS) ${INCLUDES} -o $@ -c $<

$(BUILDPATH)/%.o:%.cpp
	$(CPP) $(CFLAGS) ${INCLUDES} -o $@ -c $<

$(TARGET):$(LIBOBJ)
	$(CPP) -o $@ $^ $(LDFLAGS)

clean:
	rm -f $(TARGET) $(LIBOBJ)
