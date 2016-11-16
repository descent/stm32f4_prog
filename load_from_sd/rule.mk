CFLAGS=-g -std=c99 

ifeq ($(ARCH), cm3) 
CROSS=arm-none-eabi-
CFLAGS +=-DSTM32F407 -nostartfiles -fno-common -O0 -g -mcpu=cortex-m3 -mthumb 
else
endif

#AR=ar
#CC=gcc
#CXX=g++

AR=$(CROSS)ar
CC=$(CROSS)gcc
CXX=$(CROSS)g++

