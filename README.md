# selfdrive

> Autonomy Lab's monorepo

## Getting Started

`selfdrive` is a monorepo consisting of various Autonomy Lab projects
written in different languages with various dependencies. To avoid
dependency hell, we've opted to utilize the [Nix package manager] to
handle dependency resolution along with [direnv] to automate this
process.

### Installing Nix

Head over to <https://nixos.org/download> and proceed with the
installation for your platform. Once the process is complete, you'll
need to enable [Nix flakes] by adding the following to
`~/.config/nix/nix.conf`:

```
experimental-features = flakes nix-command
```

You'll also need to enable the unstable [channel] if you'd like to
install the latest versions of software:

```sh
$ nix-channel --add https://nixos.org/channels/nixpkgs-unstable nixpkgs
$ nix-channel --update
```

### Installing direnv

The easiest way to install [direnv] is through [`nix-env`]:

```sh
$ nix-env -iA nixpkgs.direnv
```

> Optionally, feel free to use your system's package manager to go
> through with the installation of [direnv].

Once installed, [hook direnv into your shell].

### Cloning `selfdrive`

Make sure that you've installed [Git] and [Git-LFS] before cloning.

Clone the repo:

```sh
$ git clone git@github.com:CooperUnion/selfdrive.git
```

Once cloned, make sure to initialize all the submodules:

```sh
$ git submodule update --init --recursive
```

We also recommend to configure [Git] to automatically update submodules
recursively:

```sh
$ git config --global submodule.recurse true
```

### Bootstrapping

Once everything's installed and set up, you can pass over the bootstrap
process to [direnv]:

```sh
$ direnv allow
```

If everything worked, you should be able to run [SCons] successfully:

```sh
$ scons
```

## Copyright & Licensing

Copyright (C) 2021--2023 Autonomy Lab

Distributed under the [GPLv3] only.

[channel]: https://nixos.wiki/wiki/Nix_channels
[direnv]: https://direnv.net/
[git]: https://git-scm.com/
[git-lfs]: https://git-lfs.com/
[gplv3]: LICENSE.md
[hook direnv into your shell]: https://direnv.net/docs/hook.html
[nix flakes]: https://nixos.wiki/wiki/Flakes
[nix package manager]: https://nixos.org/
[scons]: https://scons.org/
[`nix-env`]: https://nixos.org/manual/nix/stable/command-ref/nix-env
