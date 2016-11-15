#AR=ar
#CC=gcc
#CXX=g++

CROSS=arm-none-eabi-
AR=$(CROSS)ar
CC=$(CROSS)gcc
CXX=$(CROSS)g++
CFLAGS=-g -DSTM32F407 -std=c99 -nostartfiles -fno-common -O0 -g -mcpu=cortex-m3 -mthumb 

