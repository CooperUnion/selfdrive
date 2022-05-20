# CANd

`cand` (pronounced _candy_) is the solution to your CAN
send/receieve/decode/encode woes.

`cand` listens to and decodes CAN messages with a provided DBC file, and also
listens for, encodes, and sends CAN messages you ask it to send.

Run `cand` as a daemon and use `cand.client` to interact with it. For example:

```python
from cand.client import Bus

bus = Bus()
bus.send('dbwNode_Status', {'SystemStatus': 'ESTOP', 'Counter': 11})
print(bus.get('dbwNode_Encoder_Data'))
# (1652836556992745935, {'Encoder0': 133, 'Encoder1': 152, 'Time': 10000})
```

## Performance
`cand` and the `cand.client.Bus` interface are very fast. Neither `Bus.get()`
nor `Bus.send()` calls block for very long, because they simply query or write
to the Redis backend. `cand` monitors its outbound CAN send performance and
will leave log messages when the TX buffer starts to grow too large. Messages
will not be lost, however.

**Note**: cand will work through its backlog of messages to be sent regardless
of the size; we should change that behavior eventually.

## Redis
`cand` uses Redis as the storage and request backend between itself and
clients. You can specify the Redis host and port for `cand` as well as the
client.
