{
  lib,
  stdenv,
  buildPythonPackage,
  fetchFromGitHub,
  pythonOlder,
  setuptools,
  cantools,
  coloredlogs,
  msgpack,
  python-can,
  redis,
}:

buildPythonPackage rec {
  pname = "opencan-cand";
  version = "0.1.2";
  pyproject = true;

  disabled = pythonOlder "3.9";

  src = fetchFromGitHub {
    owner = "opencan";
    repo = "cand";
    rev = "d31fc48dedc25c6fb8f4f413bd1ff98b9cdc5177";
    hash = "sha256-Mvko4ia6iDXKZ3htfoZ9EoT60it0H2Bn7GXw5oNJ3bA=";
  };

  build-system = [ setuptools ];

  dependencies = [
    cantools
    coloredlogs
    msgpack
    python-can
    redis
  ];

  pythonImportsCheck = [ "cand" ];

  meta = with lib; {
    description = "Fast and super useful daemon for decoding and encoding CAN messages";
    homepage = "https://github.com/opencan/cand";
    license = licenses.mpl20;
  };
}
