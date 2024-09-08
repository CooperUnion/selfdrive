{ pkgs, ... }:

let
  python = pkgs.python312;
  pythonPackages = pkgs.python312Packages;

  rust = import ./rust.nix { inherit pkgs; };
in
[
  python
  pythonPackages.invoke
  rust
]
++ (with pkgs; [
  act
  cmake
  llvmPackages_latest.clang-unwrapped
  mdbook
  ninja
  nixfmt-rfc-style
  texliveFull
  zlib
])
