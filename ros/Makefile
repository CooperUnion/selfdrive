# tools
DOCKER         ?= docker
DOCKER_COMPOSE ?= $(DOCKER) compose


# build vars
DOCKER_FLAGS       ?=
DOCKER_FLAGS_BUILD ?= --progress plain --pull
DOCKER_FLAGS_RM    ?= --force --stop
CAND_SRC            = ../common/cand
CAND_DST            = docker/cand/cand docker/cmd_to_ctrl/cand
IGVCUTILS_SRC       = ../common/igvcutils
IGVCUTILS_DST       = docker/cmd_to_ctrl/igvcutils

CAND_SRC           ?= ../common/cand
CANTOOLS_SRC       ?= ../common/cantools
IGVCUTILS_SRC      ?= ../common/igvcutils

CAND_DST            = docker/py-steering/cand
CANTOOLS_DST       ?= docker/py-steering/cantools
IGVCUTILS_DST       = docker/py-steering/igvcutils


.PHONY: all
all: build


.PHONY: clean
clean:
	@rm -rvf $(CAND_DST) $(IGVCUTILS_DST)


.PHONY: build
build: pydeps
	$(DOCKER_COMPOSE) build $(DOCKER_FLAGS_BUILD) $(DOCKER_FLAGS) $(CONTAINER)


.PHONY: build-clean
build-clean: pydeps
	$(DOCKER_COMPOSE) build $(DOCKER_FLAGS_BUILD) --no-cache $(DOCKER_FLAGS) $(CONTAINER)


.PHONY: pydeps
pydeps:
	@rm -rf $(CAND_DST) $(CANTOOLS_DST) $(IGVCUTILS_DST)
	@echo $(CAND_DST)      | xargs -n 1 cp -r $(CAND_SRC)
	@echo $(CANTOOLS_DST)  | xargs -n 1 cp -r $(CANTOOLS_SRC)
	@echo $(IGVCUTILS_DST) | xargs -n 1 cp -r $(IGVCUTILS_SRC)
