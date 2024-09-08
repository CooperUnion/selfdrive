{
  description = "Autonomy Lab's monorepo";

  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs/nixpkgs-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    rust-overlay = {
      url = "github:oxalica/rust-overlay";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs =
    {
      self,
      flake-utils,
      ...
    }@inputs:
    flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = import inputs.nixpkgs {
          inherit system;
          overlays = [ (import inputs.rust-overlay) ];
        };

      in
      {
        formatter = pkgs.nixfmt-rfc-style;

        devShells.default = pkgs.mkShell {
          packages = import ./nix/packages { inherit inputs pkgs; };

          shellHook = ''
            export LD_LIBRARY_PATH="''${LD_LIBRARY_PATH:-}"
            export LD_LIBRARY_PATH="${pkgs.zlib}/lib:''$LD_LIBRARY_PATH"
          '';
        };
      }
    );
}
