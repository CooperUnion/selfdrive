{ final, prev }:
python-final: python-prev: {
  opencan-cand = python-final.callPackage ../pkgs/development/python-modules/opencan-cand.nix { };
}
