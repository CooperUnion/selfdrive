{
  description = "Autonomy Lab's monorepo";

  inputs = {
    flake-utils.url = "github:numtide/flake-utils";
    nixpkgs.url = "github:nixos/nixpkgs/nixpkgs-unstable";
    pyproject-nix = {
      url = "github:nix-community/pyproject.nix";
      inputs.nixpkgs.follows = "nixpkgs";
    };
    rust-overlay = {
      url = "github:oxalica/rust-overlay";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs =
    inputs:

    inputs.flake-utils.lib.eachDefaultSystem (
      system:

      let
        pkgs = import inputs.nixpkgs {
          inherit system;
          overlays = [ (import inputs.rust-overlay) ];
        };

      in
      {
        devShells.default = pkgs.mkShell {
          packages = import ./nix/packages {
            inherit (inputs) pyproject-nix;
            inherit pkgs;
          };

	  venvDir = "./.venv";
        };

        formatter = pkgs.nixfmt-rfc-style;
      }
    );
}
