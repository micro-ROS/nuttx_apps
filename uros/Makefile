-include $(TOPDIR)/.config
-include $(TOPDIR)/Make.defs
include $(APPDIR)/Make.defs

UROS_DIR = $(APPDIR)/$(CONFIG_UROS_DIR)

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

all: .built
  .PHONY: clean depend distclean preconfig
  .PRECIOUS: ../../libapps$(LIBEXT)

.built: colcon_compile
	$(call DELDIR, $(UROS_DIR)/build/extracted)
	$(Q) mkdir $(UROS_DIR)/build/extracted
	for lib in $(shell find $(UROS_DIR)/install -name '*.a'); do \
		cd $(UROS_DIR)/build/extracted && \
		$(ARCROSSDEV)ar -x $$lib && \
		cd - > /dev/null && \
		$(ARCROSSDEV)ar q $(BIN) $(UROS_DIR)/build/extracted/* && \
		rm $(UROS_DIR)/build/extracted/*; \
	done
	$(Q) touch .built

colcon_compile: 
	cat arm_toolchain.cmake.in | \
		sed "s/@CROSSDEV@/$(CROSSDEV)/g" | \
		sed "s/@NUTTX_TOPDIR@/$(subst /,\/,$(TOPDIR))/g" |\
		sed "s/@ARCH_CPU_FLAGS@/\"$(ARCHCPUFLAGS)\"/g" |\
		sed "s/@ARCH_OPT_FLAGS@/\"$(ARCHOPTIMIZATION)\"/g" \
		> arm_toolchain.cmake
	$(Q) cd $(UROS_DIR); \
		colcon build \
			--packages-ignore-regex=.*_cpp \
			--cmake-args \
			-DBUILD_SHARED_LIBS=OFF \
			-DCMAKE_POSITION_INDEPENDENT_CODE=ON \
			-DBUILD_TESTING=OFF \
			-DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
			-DCMAKE_TOOLCHAIN_FILE=$(APPDIR)/uros/arm_toolchain.cmake \
			-DCMAKE_VERBOSE_MAKEFILE=ON \

install: 

context: colcon_compile

.depend:

depend: .depend

clean:
	$(call DELDIR, $(UROS_DIR)/build)
	$(call DELDIR, $(UROS_DIR)/install)
	$(call DELDIR, $(UROS_DIR)/log)
	$(call DELFILE, $(UROS_DIR)/.built)

distclean: clean

preconfig:

-include Make.dep
