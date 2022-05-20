import msgpack

from cantools.database.can.signal import NamedSignalValue


def _deserialize_NamedSignalValue(obj):
    if "__!NamedSignalValue__" in obj:
        obj = NamedSignalValue(
            value=obj["_value"], name=obj["_name"], comments=obj["_comments"]
        )
    return obj


def _serialize_NamedSignalValue(obj):
    if isinstance(obj, NamedSignalValue):
        return {
            "__!NamedSignalValue__": True,
            "_name": obj._name,
            "_value": obj._value,
            "_comments": obj._comments,
        }
    return obj


def serialize(obj):
    return msgpack.packb(obj, default=_serialize_NamedSignalValue)


def deserialize(obj):
    return msgpack.unpackb(obj, object_hook=_deserialize_NamedSignalValue)
