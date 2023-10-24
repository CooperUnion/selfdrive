{
  description = "Autonomy Lab's monorepo";

  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs/nixpkgs-unstable";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, nixpkgs, flake-utils }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs {
          inherit system;
        };

      in
      {
        devShells.default = pkgs.mkShell {
          packages = with pkgs; [
            nixpkgs-fmt
            python311
            scons
          ];
        };
      }
    );
}
