# tools
CANTOOLS ?= cantools
PYTHON   ?= python3.9


# build vars
CAN_DB_NAME = CAN
CAN_GEN     = can_gen
CAN_YML     = can.yml
GENERATOR   = gen_dbc.py
IGVC_DBC    = igvc_can.dbc


.PHONY: all
all: $(CAN_GEN).c $(CAN_GEN).h


.PHONY: clean
clean:
	@rm -rvf $(CAN_GEN).{c,h} $(IGVC_DBC)


$(CAN_GEN).c $(CAN_GEN).h: $(IGVC_DBC)
	$(PYTHON) -m $(CANTOOLS) generate_c_source $(IGVC_DBC) \
	--database-name $(CAN_DB_NAME) \
	--use-float

	mv $(CAN_DB_NAME).c $(CAN_GEN).c
	mv $(CAN_DB_NAME).h $(CAN_GEN).h
	sed -i 's\$(CAN_DB_NAME)_H\$(shell echo $(CAN_GEN) | tr \'[a-z]\' \'[A-Z]\')_H\g' $(CAN_GEN).h
	sed -i 's\$(CAN_DB_NAME).h\$(CAN_GEN).h\g' $(CAN_GEN).c


$(IGVC_DBC): $(CAN_YML)
	$(PYTHON) $(GENERATOR)
