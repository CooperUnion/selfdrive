# tools
DOCKER         ?= docker
DOCKER_COMPOSE ?= $(DOCKER) compose


# build vars
DOCKER_FLAGS       ?=
DOCKER_FLAGS_BUILD ?= --progress plain --pull
DOCKER_FLAGS_RM    ?= --force --stop

CAND_SRC           ?= ../common/cand
IGVCUTILS_SRC      ?= ../common/igvcutils

CAND_DST            = docker/py-steering/cand
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
	@rm -rf $(CAND_DST) $(IGVCUTILS_DST)
	@cp -r $(CAND_SRC) docker/py-steering/cand
	@cp -r $(IGVCUTILS_SRC) docker/py-steering/igvcutils
