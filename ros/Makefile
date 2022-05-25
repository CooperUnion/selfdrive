# tools
DOCKER         ?= docker
DOCKER_COMPOSE ?= $(DOCKER) compose


# build vars
DOCKER_FLAGS       ?=
DOCKER_FLAGS_BUILD ?= --progress plain --pull
DOCKER_FLAGS_RM    ?= --force --stop


.PHONY: all
all: build


.PHONY: build
build:
	$(DOCKER_COMPOSE) build $(DOCKER_FLAGS_BUILD) $(DOCKER_FLAGS) $(CONTAINER)


.PHONY: build-no-cache
build-no-cache:
	$(DOCKER_COMPOSE) build $(DOCKER_FLAGS_BUILD) --no-cache $(DOCKER_FLAGS) $(CONTAINER)


.PHONY: clean
clean:
	@$(DOCKER_COMPOSE) rm $(DOCKER_FLAGS_RM) $(DOCKER_FLAGS) $(CONTAINER)
