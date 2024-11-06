{ final, prev }:
python-final: python-prev: {
  cantools = python-prev.cantools.overridePythonAttrs (_: {
    version = "37.0.7";
    src = prev.fetchFromGitHub {
      owner = "CooperUnion";
      repo = "cantools-autonomylab";
      rev = "4eb9008df23becafab0e8e257e61b8d07264dec2";
      hash = "sha256-JobJCK2C+IhxOjuKmrQgZeUWksXaa/C6xI2FCGGmcMI=";
    };
    doCheck = false;
  });
  opencan-cand = python-final.callPackage ../pkgs/development/python-modules/opencan-cand.nix { };
}
