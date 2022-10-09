# tools
DOCKER         ?= docker
DOCKER_COMPOSE ?= $(DOCKER) compose


# build vars
DOCKER_FLAGS       ?=
DOCKER_FLAGS_BUILD ?= --progress plain --pull
DOCKER_FLAGS_RM    ?= --force --stop

CANTOOLS_SRC  := ../common/cantools
IGVCUTILS_SRC := ../common/igvcutils

CANTOOLS_DST  := docker/py-steering/cantools
IGVCUTILS_DST := docker/cmd_to_ctrl/igvcutils docker/py-steering/igvcutils


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
	rm -rf $(CANTOOLS_DST) $(IGVCUTILS_DST)
	echo $(CANTOOLS_DST)  | xargs -n 1 cp -r $(CANTOOLS_SRC)
	echo $(IGVCUTILS_DST) | xargs -n 1 cp -r $(IGVCUTILS_SRC)
