from invoke import Collection

from . import (
    cargo,
    python,
)

ns = Collection()

ns.add_collection(Collection.from_module(cargo))
ns.add_collection(Collection.from_module(python))
