-include $(TOPDIR)/Make.defs

# uROS-publisher built-in application info

CONFIG_UROS_PONG_SERVER_PRIORITY ?= SCHED_PRIORITY_DEFAULT
CONFIG_UROS_PONG_SERVER_STACKSIZE ?= 65000

APPNAME = uros_pong_server
PRIORITY = $(CONFIG_UROS_PONG_SERVER_PRIORITY)
STACKSIZE = $(CONFIG_UROS_PONG_SERVER_STACKSIZE)

# uROS-publisher Example

ASRCS =
CXXSRCS = pong_server.cxx 
MAINSRC = uros_pong_server_main.cxx

CONFIG_UROS_PONG_SERVER_PROGNAME ?= uros_pong$(EXEEXT)
PROGNAME = $(CONFIG_UROS_PONG_SERVER_PROGNAME)
UROS_INCLUDES = $(shell find $(APPDIR)/$(CONFIG_UROS_DIR)/install -type d -name include)
CFLAGS += ${shell $(INCDIR) $(INCDIROPT) "$(CC)" "$(UROS_INCLUDES)"} -std=c99
CXXFLAGS += ${shell $(INCDIR) $(INCDIROPT) "$(CC)" "$(UROS_INCLUDES)"} -std=c++11

MODULE = CONFIG_UROS_PONG_SERVER

include $(APPDIR)/Application.mk
