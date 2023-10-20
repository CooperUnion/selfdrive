#!/usr/bin/env python3

import re
import sys


def main():
    close = False

    if len(sys.argv) > 1:
        if sys.argv[1] == 'c':
            close = True

    print(
        '''rostopic pub /virtual_costmap_layer/obstacles custom_msgs/Obstacles 'list: ['''
    )

    form = True

    try:
        initial = []

        if close:
            for line in sys.stdin:
                out = re.match(r'^\s\s[xy]: -?\d+\.\d+', line)

                if out is None:
                    continue

                initial.append(out.group().strip())

                if len(initial) >= 2:
                    if form:
                        print('form: [')
                        print(f'{{{initial[0]}, {initial[1]}, z: 0.0}},')
                    if not form:
                        print(f'{{{initial[0]}, {initial[1]}, z: 0.0}}')
                        print("],")
                    form = not form
                    break

        coordinates = []
        for line in sys.stdin:
            out = re.match(r'^\s\s[xy]: -?\d+\.\d+', line)

            if out is None:
                continue

            coordinates.append(out.group().strip())

            if len(coordinates) >= 2:
                if form:
                    print('form: [')
                    print(f'{{{coordinates[0]}, {coordinates[1]}, z: 0.0}},')
                if not form:
                    print(f'{{{coordinates[0]}, {coordinates[1]}, z: 0.0}}')
                    print("],")
                form = not form
                coordinates = []

    except Exception:
        pass

    if close:
        print(f'{{{initial[0]}, {initial[1]}, z: 0.0}}')

    print("]'")


if __name__ == '__main__':
    main()
