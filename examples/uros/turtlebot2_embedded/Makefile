-include $(TOPDIR)/Make.defs

# uROS-  kobuki driver 

CONFIG_UROS_EXAMPLES_KOBUKI_PRIORITY ?= SCHED_PRIORITY_DEFAULT
CONFIG_UROS_EXAMPLES_KOBUKI_STACKSIZE ?= 65000

APPNAME = kobuki 
PRIORITY = $(CONFIG_UROS_EXAMPLES_KOBUKI_PRIORITY)
STACKSIZE = $(CONFIG_UROS_EXAMPLES_KOBUKI_STACKSIZE)

# uROS- Example for turtlebot publisher

ASRCS = 
CSRCS = serial.c
CXXSRCS = kobuki_robot.cxx kobuki_protocol.cxx thin_kobuki_test.cxx thin_kobuki_file_test.cxx
MAINSRC = thin_kobuki_node.cxx 

CONFIG_UROS_EXAMPLES_KOBUKI_PROGNAME ?= kobuki$(EXEEXT)
PROGNAME = $(CONFIG_UROS_EXAMPLES_KOBUKI_PROGNAME)
UROSDIR = $(shell find $(APPDIR)/../uros_ws/install -type d -name include)
CFLAGS += ${shell $(INCDIR) $(INCDIROPT) "$(CC)" "$(UROSDIR)"} -std=c99

MODULE = CONFIG_UROS_EXAMPLES_KOBUKI

include $(APPDIR)/Application.mk
