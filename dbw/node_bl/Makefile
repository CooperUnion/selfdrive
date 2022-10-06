# tools
PIO ?= platformio

# build vars
PIO_DIR       = .pio
PIO_RUNFLAGS ?=


.PHONY: all
all: can
	$(PIO) run $(PIO_RUNFLAGS)
	find . -type f ! -name "sdkconfig.defaults" -maxdepth 1 | egrep sdkconfig | xargs rm


.PHONY: flash
flash: can
	$(PIO) run $(PIO_RUNFLAGS) --target upload
	find . -type f ! -name "sdkconfig.defaults" -maxdepth 1 | egrep sdkconfig | xargs rm


.PHONY: clean
clean:
	@rm -rvf $(PIO_DIR)


.PHONY: can
can:
	scons -D ccan
