{ pkgs, pyproject-nix, ... }:

let
  project = pyproject-nix.lib.project.loadPyproject {
    projectRoot = ../..;
  };

  python = pkgs.python3;

in
python.withPackages (project.renderers.withPackages { inherit python; })
