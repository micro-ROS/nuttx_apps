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
	
.PHONY: clean depend distclean preconfig
.PRECIOUS: ../../libapps$(LIBEXT)


arm_toolchain.cmake: $(TOPDIR)/.config arm_toolchain.cmake.in
	cat arm_toolchain.cmake.in | \
		sed "s/@CROSSDEV@/$(CROSSDEV)/g" | \
		sed "s/@NUTTX_TOPDIR@/$(subst /,\/,$(TOPDIR))/g" |\
		sed "s/@ARCH_CPU_FLAGS@/\"$(ARCHCPUFLAGS)\"/g" |\
		sed "s/@ARCH_OPT_FLAGS@/\"$(ARCHOPTIMIZATION)\"/g" \
		> arm_toolchain.cmake

ifeq (${UROS_FAST_BUILD},y)
colcon_compile:
else
colcon_compile: arm_toolchain.cmake
	$(Q) cd $(UROS_DIR); \
			colcon build \
			--packages-ignore-regex=.*_cpp \
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
