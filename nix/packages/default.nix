{ pkgs }:

let
  llvm = pkgs.llvmPackages_latest;

  python = import ./python.nix { inherit pkgs; };

  rust = import ./rust.nix { inherit pkgs; };

in
[
  llvm.clang-unwrapped
  python
  rust
]
++ (with pkgs; [
  act
  black
  can-utils
  cmake
  flock
  mdbook
  ninja
  nixfmt-rfc-style
  platformio
  pre-commit
  ruff
  scons
  texliveFull
  toml-sort
  yamlfix
  yamllint
])
