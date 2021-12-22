-include $(TOPDIR)/Make.defs

# ucs-uROS-publisher/subscriber built-in application info

CONFIG_UCS_6LOWPAN_EXAMPLE_PRIORITY ?= SCHED_PRIORITY_DEFAULT
CONFIG_UCS_6LOWPAN_EXAMPLE_STACKSIZE ?= 65000

APPNAME = ucs_6lowpan
PRIORITY = $(CONFIG_UCS_6LOWPAN_EXAMPLE_PRIORITY)
STACKSIZE = $(CONFIG_UCS_6LOWPAN_EXAMPLE_STACKSIZE)

# ucs-uROS-publisher/subscriber Example

ASRCS =
CSRCS =
MAINSRC = ucs_6lowpan_main.c 

CONFIG_UCS_6LOWPAN_EXAMPLE_PROGNAME ?= ucs_6lowpan$(EXEEXT)
PROGNAME = $(CONFIG_UCS_6LOWPAN_EXAMPLE_PROGNAME)
UROSDIR = $(shell find $(APPDIR)/microros/micro_ros_lib/include -type d -name include)
CFLAGS += ${shell $(INCDIR) $(INCDIROPT) "$(CC)" "$(UROSDIR)"} -std=c99

MODULE = CONFIG_UCS_6LOWPAN_EXAMPLE

include $(APPDIR)/Application.mk
