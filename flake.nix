{
  description = "Autonomy Lab's monorepo";

  inputs = {
    flake-utils.url = "github:numtide/flake-utils";
    nixpkgs.url = "github:nixos/nixpkgs/nixpkgs-unstable";
    rust-overlay = {
      url = "github:oxalica/rust-overlay";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs =
    { ... }@inputs:

    inputs.flake-utils.lib.eachDefaultSystem (
      system:

      let
        pkgs = import inputs.nixpkgs {
          inherit system;
          overlays = [ (import inputs.rust-overlay) ];
        };

        ld-library-path = pkgs.lib.makeLibraryPath (
          with pkgs;
          [
            zlib
          ]
        );

      in
      {
        devShells.default = pkgs.mkShell {
          packages = import ./nix/packages { inherit pkgs; };

          LD_LIBRARY_PATH = "${ld-library-path}";
        };

        formatter = pkgs.nixfmt-rfc-style;
      }
    );
}
