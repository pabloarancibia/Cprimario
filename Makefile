#
TARGET = PRIMARIO

#
ALT_DEVICE_FAMILY ?= soc_cv_av

HWLIBS_ROOT = $(SOCEDS_DEST_ROOT)/ip/altera/hps/altera_hps/hwlib

CROSS_COMPILE = arm-linux-gnueabihf-
CFLAGS = -g -Wall -std=c++11 -I$(HWLIBS_ROOT)/include -I$(HWLIBS_ROOT)/include/$(ALT_DEVICE_FAMILY) -D$(ALT_DEVICE_FAMILY)
LDFLAGS =  -g -Wall -lrt
CC = $(CROSS_COMPILE)g++
ARCH = arm

all: $(TARGET)

$(TARGET): primario.o
	$(CC) $(LDFLAGS)   $^ -o $@

%.o : %.c
	$(CC) $(CFLAGS) -c $< -o $@

.PHONY: clean
clean:
	rm -f $(TARGET) *.a *.o *~
