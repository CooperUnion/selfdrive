import cand
import time

bus = cand.client.Bus()

while True:
    bus.send('DBW_VelocityCmd', {'linearVelCmd' : 69, 'angularVelCmd' : 420})
    time.sleep(0.1)
