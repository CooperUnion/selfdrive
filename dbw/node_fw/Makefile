# tools
PIO ?= platformio


# build vars
CAN           = ../../can
PIO_RUNFLAGS ?=


ifdef MOD
	PIO_RUNFLAGS += --environment $(MOD)
endif


.PHONY: all
all: $(CAN)
	$(PIO) run $(PIO_RUNFLAGS)


.PHONY: flash
flash: $(CAN)
	$(PIO) run $(PIO_RUNFLAGS) --target upload


.PHONY: clean
clean:
	$(PIO) run $(PIO_RUNFLAGS) --silent --target clean


.PHONY: $(CAN)
$(CAN):
	@$(MAKE) -C $(CAN)
