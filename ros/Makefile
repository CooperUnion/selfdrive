# tools
DOCKER         ?= docker
DOCKER_COMPOSE ?= $(DOCKER) compose


# build vars
DOCKER_FLAGS       ?=
DOCKER_FLAGS_BUILD ?= --progress plain --pull
DOCKER_FLAGS_RM    ?= --force --stop
CAND_SRC            = ../common/cand
CAND_DST            = docker/cand/cand docker/cmd_to_ctrl/cand


.PHONY: all
all: build


.PHONY: clean
clean:
	@rm -rvf $(CAND_DST)
	@$(DOCKER_COMPOSE) rm $(DOCKER_FLAGS_RM) $(DOCKER_FLAGS) $(CONTAINER)


.PHONY: build
build: cand_deps
	$(DOCKER_COMPOSE) build $(DOCKER_FLAGS_BUILD) $(DOCKER_FLAGS) $(CONTAINER)


.PHONY: build-no-cache
build-no-cache: cand_deps
	$(DOCKER_COMPOSE) build $(DOCKER_FLAGS_BUILD) --no-cache $(DOCKER_FLAGS) $(CONTAINER)


.PHONY: cand_deps
cand_deps: $(CAND_SRC)
	@cp -r $(CAND_SRC) docker/cand/
	@cp -r $(CAND_SRC) docker/cmd_to_ctrl/
