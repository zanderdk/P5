# Target specific macros
TARGET = main
TARGET_SOURCES = \
	main.c
TOPPERS_OSEK_OIL_SOURCE = ./main.oil

# Don't modify below part
O_PATH ?= build
include ../../nxtOSEK/ecrobot/ecrobot.mak
