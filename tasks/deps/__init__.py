from invoke import Collection

from . import (
    python,
)

ns = Collection()

ns.add_collection(Collection.from_module(python))
