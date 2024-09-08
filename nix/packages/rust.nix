{ pkgs }:

let
  rust-version = pkgs.rust-bin.fromRustupToolchainFile ../../rust-toolchain.toml;

in
rust-version.override { }
