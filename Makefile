# tools
PIP    ?= pip
PYTHON ?= python3


# build vars
CAN                  = can
DBW_NODE_FW          = dbw/node_fw
INSTALL_DEPENDENCIES = .install-dependencies
LOCAL_PYTHON_LIBS    = common/cantools common/igvcutils
REQUIREMENTS_TXT     = requirements.txt


.PHONY: help
help:
	@echo "Run 'make' followed by the target you wish to build:"
	@echo " - 'all'          -- build everything"
	@echo " - '$(CAN)'          -- CAN specific"
	@echo " - '$(DBW_NODE_FW)'  -- DBW node firmware"
	@echo " - 'dependencies' -- install necessary dependencies"
	@echo " - 'clean'        -- remove all build files"
	@echo " - 'help'         -- show this help message"


.PHONY: all
all: $(CAN) $(DBW_NODE_FW) dependencies


.PHONY: clean
clean:
	@rm -rvf $(INSTALL_DEPENDENCIES)
	@$(MAKE) -C $(CAN) clean
	@$(MAKE) -C $(DBW_NODE_FW) clean


.PHONY: dependencies
dependencies: $(INSTALL_DEPENDENCIES)


.PHONY: $(CAN)
$(CAN): $(INSTALL_DEPENDENCIES)
	@$(MAKE) -C $(CAN)


.PHONY: $(DBW_NODE_FW)
$(DBW_NODE_FW): $(INSTALL_DEPENDENCIES)
	@$(MAKE) -C $(DBW_NODE_FW)


$(INSTALL_DEPENDENCIES): $(REQUIREMENTS_TXT)
	$(PYTHON) -m $(PIP) install --upgrade pip wheel
	$(PYTHON) -m $(PIP) install --requirement $(REQUIREMENTS_TXT)
	$(PYTHON) -m $(PIP) install $(LOCAL_PYTHON_LIBS)
	$(PYTHON) -m $(PIP) install platformio
	@touch $(INSTALL_DEPENDENCIES)
