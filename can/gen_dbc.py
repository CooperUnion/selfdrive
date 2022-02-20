import cantools
import cantools.database.can as candb

import os

import strictyaml as sy

VERSION = "0.001"
OUTPUT_FILENAME = "igvc_can.dbc"


def parse_signal(name: str, d: dict):
    choices = d.get("choices")

    if choices is not None:
        choices = {v: k for v, k in enumerate(choices)}  # need a dict

    width = int(d["width"])

    new = candb.Signal(
        name=name,
        start=msg_offset,
        length=width,
        comment=d["description"],
        choices=choices,
    )

    return new, width


if __name__ == "__main__":
    print(f"Cooper IGVC CAN Network Generation v{VERSION}")

    messages = []

    f = open("can.yml", "r").read()
    spec = sy.load(f).data

    for msg in spec["messages"]:
        [(name, data)] = msg.items()

        signals = []
        msg_offset = 0

        for sig in data["signals"]:
            [(sname, sdata)] = sig.items()
            new, width = parse_signal(sname, sdata)

            signals.append(new)
            msg_offset += width

        msg_width = int(msg_offset / 8)
        if msg_offset % 8 > 0:
            msg_width += 1

        messages.append(
            candb.Message(
                frame_id=int(data["id"], 16),
                name=name,
                length=msg_width,
                cycle_time=int(data["cycletime"]),
                senders=data.get("senders"),
                signals=signals,
            )
        )

    db = candb.Database(messages)

    cantools.database.dump_file(db, OUTPUT_FILENAME)

    print("")
    print("**** DONE ****")
    os.system(f"cantools dump {OUTPUT_FILENAME}")
