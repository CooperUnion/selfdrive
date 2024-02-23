from invoke import Collection

from . import (
    cargo,
    direnv,
    idf,
    python,
)

ns = Collection()

ns.add_collection(cargo)
ns.add_collection(direnv)
ns.add_collection(idf)
ns.add_collection(python)
