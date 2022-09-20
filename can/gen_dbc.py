import argparse

import cantools
import cantools.database.can as candb

import os

import strictyaml as sy

VERSION = "0.003"

def parse_signal(name: str, d: dict):
    choices = d.get("choices")

    if choices is not None:
        choices = {v: k for v, k in enumerate(choices)}  # need a dict

    width = int(d["width"])

    offset = d.get("offset")
    offset = 0 if offset is None else int(offset)

    minimum = d.get("min")
    minimum = None if minimum is None else float(minimum)
    maximum = d.get("max")
    maximum = None if maximum is None else float(maximum)

    is_signed = d.get("is_signed")
    is_signed = False if is_signed is None else bool(is_signed)

    scale = d.get("scale")
    scale = 1 if scale is None else float(scale)

    unit = d.get("unit")

    new = candb.Signal(
        name=name,
        start=msg_offset,
        length=width,
        comment=d["description"],
        choices=choices,
        offset=offset,
        minimum=minimum,
        maximum=maximum,
        is_signed=is_signed,
        scale=scale,
        unit=unit,
    )

    return new, width


if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument('--yml', type=str, required=True)
    parser.add_argument('--dbc_out', type=str, required=True)

    args = parser.parse_args()

    print(f"Cooper IGVC CAN Network Generation v{VERSION}")

    messages = []

    f = open(args.yml, "r").read()
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

        template_flag = data.get("template")
        if template_flag is not None:
            for t_id, t in enumerate(spec["templategroups"][template_flag]):
                messages.append(
                    candb.Message(
                        frame_id=int(data["id"], 16) + t_id + 1,
                        name=name + "_" + t,
                        length=msg_width,
                        cycle_time=int(data["cycletime"]),
                        # fix sender #
                        signals=signals,
                    )
                )
        elif template_flag is None:
            pass
        else:
            print('Error: value for "template" must be "yes"')
            exit(-3)

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

    cantools.database.dump_file(db, args.dbc_out)

    print("")
    print("**** DONE ****")
    os.system(f"cantools dump {args.dbc_out}")
