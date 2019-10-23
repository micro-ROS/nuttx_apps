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
CXXSRCS = kobuki_robot.cxx kobuki_protocol.cxx ros_util.cxx thin_kobuki_node.cxx
MAINSRC = kobuki_main.cxx

CONFIG_UROS_EXAMPLES_KOBUKI_PROGNAME ?= kobuki$(EXEEXT)
PROGNAME = $(CONFIG_UROS_EXAMPLES_KOBUKI_PROGNAME)
UROS_INCLUDES = $(shell find $(APPDIR)/$(CONFIG_UROS_DIR)/install -type d -name include)
CFLAGS += ${shell $(INCDIR) $(INCDIROPT) "$(CC)" "$(UROS_INCLUDES)"} -std=c99
CXXFLAGS += ${shell $(INCDIR) $(INCDIROPT) "$(CC)" "$(UROS_INCLUDES)"} -std=c++11
MODULE = CONFIG_UROS_EXAMPLES_KOBUKI

include $(APPDIR)/Application.mk
