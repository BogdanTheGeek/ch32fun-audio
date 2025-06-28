all : flash

TARGET:=main
TARGET_MCU:=CH32V006
CH32V003FUN ?= ./ch32fun/ch32fun

ADDITIONAL_C_FILES += $(filter-out $(TARGET).c, $(wildcard *.c))

include $(CH32V003FUN)/ch32fun.mk

flash : cv_flash
clean : cv_clean
	rm ./tools/wav2code

format:
	clang-format --style=file -i *.c *.h

./tools/wav2code: ./tools/wav2code.c
	cc -o $@ $<

.PHONY: tools
tools: ./tools/wav2code

