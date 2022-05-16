import logging
import redis

from cand.serialization import serialize, deserialize

REDIS_SOCKET_CONNECT_TIMEOUT = 1


class Client:
    def __init__(self, redis_host: str = "localhost", redis_port: str = "6379"):

        self._log = logging.getLogger("candclient")

        try:
            self._rdb = redis.Redis(
                host=redis_host,
                port=redis_port,
                socket_connect_timeout=REDIS_SOCKET_CONNECT_TIMEOUT,
            )
        except redis.ConnectionError as e:
            self.log.error(f"Failed to connect to Redis: {e}")
            exit(-1)

    def get(self, name: str) -> dict:
        try:
            data = self._rdb.get(name)

            if data is None:
                self._log.debug(f"Tried to get message '{name}' and it was unavailable")
                return None

            data = deserialize(data)

            return data

        except Exception as e:
            self._log.error(f"Error getting message {name}: {e}")
            raise e

    def send(self, name: str, data: dict) -> None:
        try:
            data = serialize((name, data))

            self._rdb.rpush("queue:cansend", data)

        except Exception as e:
            self._log.error(f"Error sending message {name} with data {data}: {e}")
            raise e
