{
  description = "Autonomy Lab's monorepo";

  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs/nixpkgs-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    rust-overlay = {
      url = "github:oxalica/rust-overlay";
      inputs = {
        nixpkgs.follows = "nixpkgs";
        flake-utils.follows = "flake-utils";
      };
    };
  };

  outputs = { self, nixpkgs, flake-utils, rust-overlay }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ (import rust-overlay) ];
        };

        python = pkgs.python311;

        rust-version = pkgs.rust-bin.fromRustupToolchainFile ./rust-toolchain.toml;
        rust = rust-version.override { };

      in
      {
        devShells.default = pkgs.mkShell {
          packages = [
            pkgs.act
            pkgs.mdbook
            pkgs.nixpkgs-fmt
            python
            rust
          ];
        };
      }
    );
}