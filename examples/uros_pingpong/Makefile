-include $(TOPDIR)/Make.defs

#Set the scheduler priority for the app.
CONFIG_UROS_PINGPONG_EXAMPLE_PRIORITY ?= SCHED_PRIORITY_DEFAULT
#Set the stack size to the app. The minimum stack size on NuttX for a micro-ROS App is 65000 bytes
CONFIG_UROS_PINGPONG_EXAMPLE_STACKSIZE ?= 65000

#This is the name of the app on the NSH console
APPNAME = uros_pingpong
PRIORITY = $(CONFIG_UROS_PINGPONG_EXAMPLE_PRIORITY)
STACKSIZE = $(CONFIG_UROS_PINGPONG_EXAMPLE_STACKSIZE)

#Add the source files.
ASRCS =
CSRCS =
MAINSRC = uros_pingpong_main.c 

CONFIG_UROS_PINGPONG_EXAMPLE_PROGNAME ?= uros_pingpong$(EXEEXT)
PROGNAME = $(CONFIG_UROS_PINGPONG_EXAMPLE_PROGNAME)
UROS_PINGPONG_INCLUDES = $(shell find $(APPDIR)/$(CONFIG_UROS_DIR)/install -type d -name include)
CFLAGS += ${shell $(INCDIR) $(INCDIROPT) "$(CC)" "$(UROS_PINGPONG_INCLUDES)"} -std=c99

MODULE = CONFIG_UROS_PINGPONG_EXAMPLE

include $(APPDIR)/Application.mk