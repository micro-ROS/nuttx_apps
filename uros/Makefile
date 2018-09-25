-include $(TOPDIR)/.config
-include $(TOPDIR)/Make.defs
include $(APPDIR)/Make.defs

UROS_DIR = $(APPDIR)/../ros2_ws

ifeq ($(CONFIG_WINDOWS_NATIVE),y)
  BIN = ..\libapps$(LIBEXT)
else
ifeq ($(WINTOOL),y)
  BIN = ..\\libapps$(LIBEXT)
else
  BIN = ../libapps$(LIBEXT)
endif
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
		cd - && \
		$(ARCROSSDEV)ar q $(BIN) $(UROS_DIR)/build/extracted/*.o && \
		rm $(UROS_DIR)/build/extracted/*.o; \
	done
	$(Q) touch .built

colcon_compile: 
	$(Q) cd $(UROS_DIR); \
		colcon build --cmake-args -DBUILD_SHARED_LIBS=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DBUILD_TESTING=OFF -DCHECK_ENDIANNESS=OFF -DCMAKE_TOOLCHAIN_FILE=$(UROS_DIR)/arm_toolchain.cmake

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
