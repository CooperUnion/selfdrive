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

        python = pkgs.python312;
        pythonPackages = pkgs.python312Packages;

        rust-version = pkgs.rust-bin.fromRustupToolchainFile ./rust-toolchain.toml;
        rust = rust-version.override { };

      in
      {
        devShells.default = pkgs.mkShell {
          packages = [
            pkgs.act
            pkgs.cmake
            pkgs.mdbook
            pkgs.ninja
            pkgs.nixpkgs-fmt
            pkgs.zlib
            python
            pythonPackages.invoke
            rust
          ];

          shellHook = ''
            export LD_LIBRARY_PATH="''${LD_LIBRARY_PATH:-}"
            export LD_LIBRARY_PATH="${pkgs.zlib}/lib:''$LD_LIBRARY_PATH"
          '';
        };
      }
    );
}
