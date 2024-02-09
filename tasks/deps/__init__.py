from invoke import Collection

from . import (
    cargo,
    idf,
    python,
)

ns = Collection()

ns.add_collection(Collection.from_module(cargo))
ns.add_collection(Collection.from_module(idf))
ns.add_collection(Collection.from_module(python))
