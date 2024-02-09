from invoke import Collection

from . import (
    cargo,
    idf,
    python,
)

ns = Collection()

ns.add_collection(cargo)
ns.add_collection(idf)
ns.add_collection(python)
