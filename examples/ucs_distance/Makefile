-include $(TOPDIR)/Make.defs

# uROS-distance-publisher-6lowpan built-in application info

CONFIG_UCS_DISTANC_EXAMPLE_PRIORITY ?= SCHED_PRIORITY_DEFAULT
CONFIG_UCS_DISTANC_EXAMPLE_STACKSIZE ?= 65000

APPNAME = ucs_distance
PRIORITY = $(CONFIG_UCS_DISTANC_EXAMPLE_PRIORITY)
STACKSIZE = $(CONFIG_UCS_DISTANC_EXAMPLE_STACKSIZE)

# uROS-publisher Example

ASRCS =
CSRCS = init_distance_6lowpan.c
MAINSRC = ucs_distance_main.c 

CONFIG_UCS_DISTANC_EXAMPLE_PROGNAME ?= ucs_distance$(EXEEXT)
PROGNAME = $(CONFIG_UCS_DISTANC_EXAMPLE_PROGNAME)
UCS_DISTANCE_INCLUDES = $(shell find $(APPDIR)/microros/micro_ros_lib/include -type d -name include)
CFLAGS += ${shell $(INCDIR) $(INCDIROPT) "$(CC)" "$(UCS_DISTANCE_INCLUDES)"} -std=c99

MODULE = CONFIG_UCS_DISTANC_EXAMPLE

include $(APPDIR)/Application.mk
