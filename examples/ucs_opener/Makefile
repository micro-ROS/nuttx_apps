-include $(TOPDIR)/Make.defs

# uROS-publisher built-in application info

CONFIG_UCS_OPENER_EXAMPLE_PRIORITY ?= SCHED_PRIORITY_DEFAULT
CONFIG_UCS_OPENER_EXAMPLE_STACKSIZE ?= 65000

APPNAME = ucs_opener
PRIORITY = $(CONFIG_UCS_OPENER_EXAMPLE_PRIORITY)
STACKSIZE = $(CONFIG_UCS_OPENER_EXAMPLE_STACKSIZE)

# uROS-publisher Example

ASRCS =
CSRCS = init_opener_6lowpan.c
MAINSRC = ucs_opener_main.c

CONFIG_UCS_OPENER_EXAMPLE_PROGNAME ?= ucs_opener$(EXEEXT)
PROGNAME = $(CONFIG_UCS_OPENER_EXAMPLE_PROGNAME)
UCS_OPENER_INCLUDES = $(shell find $(APPDIR)/microros/micro_ros_lib/include -type d -name include)
CFLAGS += ${shell $(INCDIR) $(INCDIROPT) "$(CC)" "$(UCS_OPENER_INCLUDES)"} -std=c99

MODULE = CONFIG_UCS_OPENER_EXAMPLE

include $(APPDIR)/Application.mk
