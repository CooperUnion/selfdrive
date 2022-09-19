# tools
PIO ?= platformio


# build vars
CAN           = ../../can
PIO_DIR       = .pio
PIO_RUNFLAGS ?=


ifdef MOD
	PIO_RUNFLAGS += --environment $(MOD)
endif


.PHONY: all
all: $(CAN)
	$(PIO) run $(PIO_RUNFLAGS)
	find . -type f ! -name "sdkconfig.defaults" -maxdepth 1 | egrep sdkconfig | xargs rm


.PHONY: flash
flash: $(CAN)
	$(PIO) run $(PIO_RUNFLAGS) --target upload
	find . -type f ! -name "sdkconfig.defaults" -maxdepth 1 | egrep sdkconfig | xargs rm


.PHONY: clean
clean:
	@rm -rvf $(PIO_DIR)


.PHONY: $(CAN)
$(CAN):
	@$(MAKE) -C $(CAN)
