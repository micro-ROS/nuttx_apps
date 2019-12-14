-include $(TOPDIR)/.config
-include $(TOPDIR)/Make.defs
include $(APPDIR)/Make.defs

UROS_DIR = $(APPDIR)/$(CONFIG_UROS_DIR)


# Build type flag
ifeq ($(CONFIG_DEBUG_SYMBOLS),y)
	BUILD_TYPE = Debug
else
	BUILD_TYPE = RelWithDebInfo
endif

all: colcon_compile
	$(MAKE) -f Makefile.apps

COLCON_BUILD_ARGS = \
	--packages-ignore-regex=.*_cpp \
	--cmake-args \
	-DBUILD_SHARED_LIBS=OFF \
	-DCMAKE_POSITION_INDEPENDENT_CODE=ON \
	-DBUILD_TESTING=OFF \
	-DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
	-DCMAKE_TOOLCHAIN_FILE=$(APPDIR)/uros/arm_toolchain.cmake \
	-DCMAKE_VERBOSE_MAKEFILE=ON \
	
ifeq ($(CONFIG_MICRO_XRCEDDS_CUSTOM_CONFIG),y)
	ifeq ($(CONFIG_MICRO_XRCEDDS_TRANSPORT_SERIAL),y)
		MICRO_XRCEDDS_TRANSPORT = serial
	endif
	ifeq ($(CONFIG_MICRO_XRCEDDS_TRANSPORT_UDP),y)
		MICRO_XRCEDDS_TRANSPORT = udp
	endif
	ifeq ($(CONFIG_MICRO_XRCEDDS_CREATION_MODE_XML),y)
		MICRO_XRCEDDS_CREATION_MODE = xml
	endif
	ifeq ($(CONFIG_MICRO_XRCEDDS_CREATION_MODE_REFS),y)
		MICRO_XRCEDDS_CREATION_MODE = refs
	endif
	
# 	MICRO_XRCEDDS_CONFIG = -DMICRO_XRCEDDS_CONFIG_PATH=$(abspath rmw_microxrcedds.config)
colcon_compile: rmw_microxrcedds.config
COLCON_BUILD_ARGS += \
	-DMICRO_XRCEDDS_CONFIG_PATH=$(abspath rmw_microxrcedds.config)
endif

.PHONY: clean depend distclean preconfig
.PRECIOUS: ../../libapps$(LIBEXT)

rmw_microxrcedds.config: $(TOPDIR)/.config rmw_microxrcedds.config.in
	cat rmw_microxrcedds.config.in  \
		| sed "s/@TRANSPORT@/$(MICRO_XRCEDDS_TRANSPORT)/g" \
		| sed "s/@DEVICE@/$(subst ",\",$(subst /,\/,$(CONFIG_MICRO_XRCEDDS_TRANSPORT_SERIAL_DEVICE)))/g" \
		| sed "s/@IP@/$(subst ",\",$(CONFIG_MICRO_XRCEDDS_TRANSPORT_UDP_IP))/g" \
		| sed "s/@PORT@/$(CONFIG_MICRO_XRCEDDS_TRANSPORT_UDP_PORT)/g" \
		| sed "s/@CREATION_MODE@/$(MICRO_XRCEDDS_CREATION_MODE)/g" \
		| sed "s/@HISTORY@/$(CONFIG_MICRO_XRCEDDS_MAX_HISTORY)/g" \
		| sed "s/@NODES@/$(CONFIG_MICRO_XRCEDDS_MAX_NODES)/g" \
		| sed "s/@PUBLISHERS_X_NODE@/$(CONFIG_MICRO_XRCEDDS_MAX_PUBLISHERS_X_NODE)/g" \
		| sed "s/@SUBSCRIPTIONS_X_NODE@/$(CONFIG_MICRO_XRCEDDS_MAX_SUBSCRIPTIONS_X_NODE)/g" \
		| sed "s/@NODE_NAME_MAX@/$(CONFIG_MICRO_XRCEDDS_RMW_NODE_NAME_MAX_NAME_LENGTH)/g" \
		| sed "s/@TOPIC_NAME_MAX@/$(CONFIG_MICRO_XRCEDDS_RMW_TOPIC_NAME_MAX_NAME_LENGTH)/g" \
		| sed "s/@TYPE_NAME_MAX@/$(CONFIG_MICRO_XRCEDDS_RMW_TYPE_NAME_MAX_NAME_LENGTH)/g" \
		> rmw_microxrcedds.config

arm_toolchain.cmake: $(TOPDIR)/.config arm_toolchain.cmake.in
	cat arm_toolchain.cmake.in \
		| sed "s/@CROSSDEV@/$(CROSSDEV)/g" \
		| sed "s/@NUTTX_TOPDIR@/$(subst /,\/,$(TOPDIR))/g" \
		| sed "s/@BOARD@/$(CONFIG_ARCH_BOARD)/g" \
		| sed "s/@ARCH_CPU_FLAGS@/\"$(ARCHCPUFLAGS)\"/g" \
		| sed "s/@ARCH_OPT_FLAGS@/\"$(ARCHOPTIMIZATION)\"/g" \
		> arm_toolchain.cmake
		
# ifeq ($(CONFIG_MICRO_XRCEDDS_CUSTOM_CONFIG),y)
# colcon_compile: rmw_microxrcedds.config
# endif

colcon_compile: arm_toolchain.cmake
	@echo MICRO_XRCEDDS_CONFIG=${MICRO_XRCEDDS_CONFIG}
	$(Q) cd $(UROS_DIR); \
		colcon build ${COLCON_BUILD_ARGS}

install: 

context: arm_toolchain.cmake colcon_compile

.depend:

depend: .depend 

clean:
	cd $(UROS_DIR); colcon build --cmake-target clean --cmake-target-skip-unavailable
	rm -f $(ARCHIVES:.a=.stamp_archive)
	rm -f $(ARCHIVES:.a=.stamp_update)
	rm -rf extract

distclean:
	$(call DELDIR, $(UROS_DIR)/build)
	$(call DELDIR, $(UROS_DIR)/install)
	$(call DELDIR, $(UROS_DIR)/log)
	$(call DELFILE, $(UROS_DIR)/.built)


preconfig:

-include Make.dep
