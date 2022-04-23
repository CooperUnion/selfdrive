# tools
PIO ?= platformio


# build vars
CAN           = ../../can
PIO_DIR       = .pio


.PHONY: all
all: $(CAN)
	$(PIO) run


.PHONY: flash
flash: $(CAN)
	$(PIO) run --target upload


.PHONY: clean
clean:
	@rm -rvf $(PIO_DIR)


.PHONY: $(CAN)
$(CAN):
	@$(MAKE) -C $(CAN)
