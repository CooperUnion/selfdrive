# tools
PIP    ?= pip
PYTHON ?= python3.9


# build vars
CAN                  = can
DBW_NODE_FW          = dbw/node_fw
DBW_NODE_BL          = dbw/node_bl
ROS                  = ros
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
	@echo " - '$(ROS)'          -- ROS specific"
	@echo " - 'help'         -- show this help message"


.PHONY: all
all: $(CAN) $(DBW_NODE_FW) $(ROS) dependencies


.PHONY: clean
clean:
	@rm -rvf $(INSTALL_DEPENDENCIES)
	scons --clean ccan
	@$(MAKE) -C $(DBW_NODE_FW) clean
	@$(MAKE) -C $(ROS) clean


.PHONY: dependencies
dependencies: $(INSTALL_DEPENDENCIES)


.PHONY: $(CAN)
$(CAN):
	scons ccan


.PHONY: $(DBW_NODE_FW)
$(DBW_NODE_FW): $(INSTALL_DEPENDENCIES)
	@$(MAKE) -C $(DBW_NODE_FW)


.PHONY: $(DBW_NODE_BL)
$(DBW_NODE_BL): $(INSTALL_DEPENDENCIES)
	@$(MAKE) -C $(DBW_NODE_BL)


.PHONY: $(ROS)
$(ROS): $(INSTALL_DEPENDENCIES)
	@$(MAKE) -C $(ROS)


$(INSTALL_DEPENDENCIES): $(REQUIREMENTS_TXT)
	$(PYTHON) -m $(PIP) install --upgrade pip wheel
	$(PYTHON) -m $(PIP) install --upgrade $(LOCAL_PYTHON_LIBS)
	$(PYTHON) -m $(PIP) install --requirement $(REQUIREMENTS_TXT)
	cargo install --root build/cargo --locked --git https://github.com/opencan/opencan --rev 4e6953fa
	@touch $(INSTALL_DEPENDENCIES)
