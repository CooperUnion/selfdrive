import logging
import redis

from time import time_ns

from cand.serialization import serialize, deserialize

REDIS_SOCKET_CONNECT_TIMEOUT = 1


class Bus:
    def __init__(self, redis_host: str = "localhost", redis_port: str = "6379"):

        self._log = logging.getLogger("candclient")

        try:
            self._rdb = redis.Redis(
                host=redis_host,
                port=redis_port,
                socket_connect_timeout=REDIS_SOCKET_CONNECT_TIMEOUT,
            )
        except redis.ConnectionError as e:
            self._log.error(f"Failed to connect to Redis: {e}")
            exit(-1)

    def get(self, name: str) -> tuple[int, dict]:
        try:
            data = self._rdb.get(name)

            if data is None:
                self._log.debug(f"Tried to get message '{name}' and it was unavailable")
                return None

            data = deserialize(data)

            return tuple(data)

        except Exception as e:
            self._log.error(f"Error getting message {name}: {e}")
            raise e

    def get_data(self, name: str) -> dict:
        data = self.get(name)

        if data is not None:
            return data[1]

    def get_time(self, name: str) -> int:
        data = self.get(name)

        if data is not None:
            return data[0]

    def get_time_delta(self, name: str) -> int:
        data = self.get(name)

        if data is not None:
            return time_ns() - data[0]

    def send(self, name: str, data: dict) -> None:
        try:
            data = serialize((name, data))

            self._rdb.rpush("queue:cansend", data)

        except Exception as e:
            self._log.error(f"Error sending message {name} with data {data}: {e}")
            raise e
