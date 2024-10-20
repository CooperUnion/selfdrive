{ pkgs, ... }@inputs:

let
  llvm = pkgs.llvmPackages_latest;

  python = import ./python.nix {
    inherit (inputs) pyproject-nix;
    inherit pkgs;
  };

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
  mdbook
  ninja
  nixfmt-rfc-style
  platformio
  pre-commit
  python3Packages.venvShellHook
  ruff
  texliveFull
  toml-sort
  yamlfix
  yamllint
  zlib
])
