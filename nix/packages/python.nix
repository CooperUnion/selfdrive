{ pkgs }:

let

  python = pkgs.python3;

in
python.withPackages (
  python-pkgs: with python-pkgs; [
    can-isotp
    cantools
    invoke
    matplotlib
    mdformat-gfm
    opencan-cand
    schema
    strictyaml
    tqdm
  ]
)
