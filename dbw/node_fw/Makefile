# tools
PIO ?= platformio


# build vars
PIO_DIR       = .pio
PIO_RUNFLAGS ?=


ifdef MOD
	PIO_RUNFLAGS += --environment $(MOD)
endif


.PHONY: all
all: can
	$(PIO) run $(PIO_RUNFLAGS)


.PHONY: flash
flash: can
	$(PIO) run $(PIO_RUNFLAGS) --target upload


.PHONY: clean
clean:
	@rm -rvf $(PIO_DIR)


.PHONY: can
can:
	scons -D ccan
