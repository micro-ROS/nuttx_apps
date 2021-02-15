-include $(TOPDIR)/.config
-include $(TOPDIR)/Make.defs
include $(APPDIR)/Make.defs

MICRO_XRCE_DIR = $(APPDIR)/microxrcedds/Micro-XRCE-DDS-Client
MICRO_XRCE_INSTALL_DIR = $(MICRO_XRCE_DIR)/build/install

ifeq ($(CONFIG_WINDOWS_NATIVE),y)
  BIN = ..\libapps$(LIBEXT)
else
ifeq ($(WINTOOL),y)
  BIN = ..\\libapps$(LIBEXT)
else
  BIN = ../libapps$(LIBEXT)
endif
endif

# Build type flag
ifeq ($(CONFIG_DEBUG_SYMBOLS),y)
	BUILD_TYPE = Debug
else
	BUILD_TYPE = Release
endif

# Create Entities by REF profile flag
ifeq ($(CONFIG_MR_PROFILE_CREATE_ENTITIES_REF),y)
	PROFILE_CREATE_ENTITIES_REF = TRUE
else
	PROFILE_CREATE_ENTITIES_REF = FALSE
endif

# Create Entities by XML profile flag
ifeq ($(CONFIG_MR_PROFILE_CREATE_ENTITIES_XML),y)
	PROFILE_CREATE_ENTITIES_XML = TRUE
else
	PROFILE_CREATE_ENTITIES_XML = FALSE
endif

# Read Access profile flag
ifeq ($(CONFIG_MR_PROFILE_READ_ACCESS),y)
	PROFILE_READ_ACCESS = TRUE
else
	PROFILE_READ_ACCESS = FALSE
endif

# Serial Transport profile flag
ifeq ($(CONFIG_MR_PROFILE_SERIAL_TRANSPORT),y)
	PROFILE_SERIAL_TRANSPORT = TRUE
else
	PROFILE_SERIAL_TRANSPORT = FALSE
endif

# UDP Transport profile flag
ifeq ($(CONFIG_MR_PROFILE_UDP_TRANSPORT),y)
	PROFILE_UDP_TRANSPORT = TRUE
else
	PROFILE_UDP_TRANSPORT = FALSE
endif

# TCP Transport profile flag
ifeq ($(CONFIG_MR_PROFILE_TCP_TRANSPORT),y)
	PROFILE_TCP_TRANSPORT = TRUE
else
	PROFILE_TCP_TRANSPORT = FALSE
endif

# Serialization endianness flag
ifeq ($(CONFIG_ENDIAN_BIG),y)
	CONFIG_SERIALIZATION_ENDIANNESS = 1
else
	CONFIG_SERIALIZATION_ENDIANNESS = 0
endif

# Verbose flag
ifeq ($(CONFIG_MICRO_XRCE_VERBOSE_OUTPUT),y)
	VERBOSE_OPT = ON
else
	VERBOSE_OPT = OFF
endif

all: .built
  .PHONY: clean depend distclean preconfig
  .PRECIOUS: ../../libapps$(LIBEXT)

.built: cmakecompile 
	$(call DELDIR, $(MICRO_XRCE_DIR)/build/extracted)
	$(Q) mkdir $(MICRO_XRCE_DIR)/build/extracted
	$(Q) cd $(MICRO_XRCE_DIR)/build/extracted; \
		$(ARCROSSDEV)ar -x $(MICRO_XRCE_INSTALL_DIR)/lib/libmicroxrcedds_client.a; \
		$(ARCROSSDEV)ar -x $(MICRO_XRCE_INSTALL_DIR)/lib/libmicrocdr.a;
	$(Q) $(AR) $(BIN) $(MICRO_XRCE_DIR)/build/extracted/*.o
	$(Q) touch .built

cmakecompile: $(MICRO_XRCE_DIR) $(MICRO_XRCE_INSTALL_DIR)/lib/libmicroxrcedds_client.a

$(MICRO_XRCE_DIR):
	@echo "Cloning eProsima/Micro-XRCE-DDS-Client"	
	$(Q) git clone  https://github.com/eProsima/Micro-XRCE-DDS-Client

$(MICRO_XRCE_INSTALL_DIR)/lib/libmicroxrcedds_client.a: $(MICRO_XRCE_DIR)/build/CMakeCache.txt
	$(Q) cd $(MICRO_XRCE_DIR)/build; \
		 make install

$(MICRO_XRCE_DIR)/build/CMakeCache.txt: $(TOPDIR)/.config
	$(Q) mkdir -p $(MICRO_XRCE_DIR)/build
	$(Q) cd $(MICRO_XRCE_DIR)/build; \
		 cmake \
			-DARCH_CPU_FLAGS="$(ARCHCPUFLAGS)" \
			-DARCH_OPT_FLAGS="$(ARCHOPTIMIZATION)" \
			-DCHECK_ENDIANNESS=OFF \
			-DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
			-DCMAKE_INSTALL_PREFIX=$(MICRO_XRCE_INSTALL_DIR) \
			-DCMAKE_SYSROOT=$(TOPDIR) \
			-DCMAKE_TOOLCHAIN_FILE=$(MICRO_XRCE_DIR)/../toolchain.cmake \
			-DCROSSDEV=$(CROSSDEV) \
			-DUCLIENT_BIG_ENDIANNESS=$(CONFIG_SERIALIZATION_ENDIANNESS) \
			-DUCLIENT_PROFILE_UDP=$(PROFILE_UDP_TRANSPORT) \
			-DUCLIENT_PROFILE_TCP=$(PROFILE_TCP_TRANSPORT) \
			-DUCLIENT_PROFILE_SERIAL=$(PROFILE_SERIAL_TRANSPORT) \
			-DUCLIENT_MAX_OUTPUT_BEST_EFFORT_STREAMS=$(CONFIG_MAX_OUTPUT_BEST_EFFORT_STREAMS) \
			-DUCLIENT_MAX_OUTPUT_RELIABLE_STREAMS=$(CONFIG_MAX_OUTPUT_RELIABLE_STREAMS) \
			-DUCLIENT_MAX_INPUT_BEST_EFFORT_STREAMS=$(CONFIG_MAX_INPUT_BEST_EFFORT_STREAMS) \
			-DUCLIENT_MAX_INPUT_RELIABLE_STREAMS=$(CONFIG_MAX_INPUT_RELIABLE_STREAMS) \
			-DUCLIENT_MAX_SESSION_CONNECTION_ATTEMPTS=$(CONFIG_MAX_SESSION_CONNECTION_ATTEMPTS) \
			-DUCLIENT_MIN_SESSION_CONNECTION_INTERVAL=$(CONFIG_MIN_SESSION_CONNECTION_INTERVAL) \
			-DUCLIENT_MIN_HEARTBEAT_TIME_INTERVAL=$(CONFIG_MIN_HEARTBEAT_TIME_INTERVAL) \
			-DUCLIENT_UDP_TRANSPORT_MTU=$(CONFIG_MR_UDP_TRANSPORT_MTU) \
			-DUCLIENT_TCP_TRANSPORT_MTU=$(CONFIG_MR_TCP_TRANSPORT_MTU) \
			-DUCLIENT_SERIAL_TRANSPORT_MTU=$(CONFIG_MR_SERIAL_TRANSPORT_MTU) \
			-DVERBOSE_SERIALIZATION=$(VERBOSE_OPT) \
			-DVERBOSE_MESSAGE=$(VERBOSE_OPT) \
			..; \

install: 
	$(Q) cd $(MICRO_XRCE_DIR)/build/; \
		 make install

context: cmakecompile 

.depend:

depend: .depend

clean:
	$(call DELDIR, $(MICRO_XRCE_DIR)/build)
	$(call DELFILE, $(MICRO_XRCE_DIR)/.built)

distclean: clean
	$(call DELDIR, $(MICRO_XRCE_DIR))

preconfig:

-include Make.dep
