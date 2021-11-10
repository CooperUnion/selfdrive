# Node Firmware


## Docker Build Container

In an effort to create a consistent build environment, we've created a docker
container for building firmware.  To get up and running simply run:

```bash
docker run \
	--interactive \
	--tty \
	--rm \
	--volume $(pwd):/build \
	git-registry.cooperigvc.org/igvc/selfdrive/dbw/node_fw
```
