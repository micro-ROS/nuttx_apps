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

ifeq ($(CONFIG_UROS_TRANSPORT_UDP),y)
  RMW_UXRCE_TRANSPORT=udp
  AGENT_IP=$(CONFIG_UROS_AGENT_IP)
else
  RMW_UXRCE_TRANSPORT=serial
  RMW_UXRCE_SERIAL_DEVICE=$(shell echo $(CONFIG_UROS_SERIAL_PORT) |\
  	sed 's/\"//g')
endif



all: colcon_compile
	$(MAKE) -f Makefile.apps
	
.PHONY: clean depend distclean preconfig
.PRECIOUS: ../../libapps$(LIBEXT)


arm_toolchain.cmake: $(TOPDIR)/.config arm_toolchain.cmake.in
	cat arm_toolchain.cmake.in | \
		sed "s/@CROSSDEV@/$(CROSSDEV)/g" | \
		sed "s/@NUTTX_TOPDIR@/$(subst /,\/,$(TOPDIR))/g" |\
		sed "s/@ARCH_CPU_FLAGS@/\"$(ARCHCPUFLAGS)$(UROS_INST_FLAG)\"/g" |\
		sed "s/@ARCH_OPT_FLAGS@/\"$(ARCHOPTIMIZATION)\"/g" \
		> arm_toolchain.cmake

rmw_config.meta: $(TOPDIR)/.config rmw_config.meta.in
	cat rmw_config.meta.in | \
		sed "s/@MAX_NODES@/$(CONFIG_UROS_MAX_NODES)/g" | \
		sed "s/@MAX_PUBLISHERS@/$(CONFIG_UROS_MAX_PUBLISHERS)/g" | \
		sed "s/@MAX_SUBSCRIPTIONS@/$(CONFIG_UROS_MAX_SUBSCRIPTIONS)/g" | \
		sed "s/@MAX_SERVICES@/$(CONFIG_UROS_MAX_SERVICES)/g" | \
		sed "s/@MAX_CLIENTS@/$(CONFIG_UROS_MAX_CLIENTS)/g" | \
		sed "s/@MAX_HISTORY@/$(CONFIG_UROS_MAX_HISTORY)/g" | \
		sed "s/@TRANSPORT@/$(RMW_UXRCE_TRANSPORT)/g" | \
		sed "s/@AGENT_IP@/$(AGENT_IP)/g" | \
		sed "s/@AGENT_PORT@/$(CONFIG_UROS_AGENT_PORT)/g" | \
		sed 's#@SERIAL_PORT@#$(RMW_UXRCE_SERIAL_DEVICE)#g' | \
		sed "s/@IP_VERSION@/$(CONFIG_UROS_IP_VERSION)/g" \
		> rmw_config.meta

ifeq (${UROS_FAST_BUILD},y)
colcon_compile:
else
colcon_compile: arm_toolchain.cmake rmw_config.meta
	$(Q) cd $(UROS_DIR); \
			colcon build \
			--packages-ignore-regex=.*_cpp \
			--metas $(UROS_DIR)/colcon.meta $(APPDIR)/uros/rmw_config.meta \
			--cmake-args \
			-DBUILD_SHARED_LIBS=OFF \
			-DCMAKE_POSITION_INDEPENDENT_CODE=ON \
			-DBUILD_TESTING=OFF \
			-DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
			-DCMAKE_TOOLCHAIN_FILE=$(APPDIR)/uros/arm_toolchain.cmake \
			-DCMAKE_VERBOSE_MAKEFILE=ON
endif

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
