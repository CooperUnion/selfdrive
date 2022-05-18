import time

from cand.client import Bus


def main():
    bus = Bus()

    iterations = 10000
    i = 0
    start_time = time.time()
    while i - 1 < 100000000:
        bus.send("dbwNode_Status", {"SystemStatus": "ESTOP", "Counter": 141})
        # time.sleep(0.00000001)

        if not i % iterations:
            print(
                f"Messages sent / sec: {int(iterations / (time.time() - start_time))}, total: {i:,}"
            )
            start_time = time.time()

        i += 1


if __name__ == "__main__":
    main()
