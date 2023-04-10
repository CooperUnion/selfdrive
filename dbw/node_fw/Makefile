# tools
PIO ?= platformio


# build vars
PIO_DIR       = ../../build/dbw/node_fw
PIO_RUNFLAGS ?=


ifdef MOD
	PIO_RUNFLAGS += --environment $(MOD)
endif


.PHONY: all
all:
	$(PIO) run $(PIO_RUNFLAGS)
	find . -type f ! -name "sdkconfig.defaults" -maxdepth 1 | egrep sdkconfig | xargs rm


.PHONY: flash
flash:
	$(PIO) run $(PIO_RUNFLAGS) --target upload
	find . -type f ! -name "sdkconfig.defaults" -maxdepth 1 | egrep sdkconfig | xargs rm


.PHONY: clean
clean:
	@rm -rvf $(PIO_DIR)
