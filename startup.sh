#!/bin/bash


cand --dev can0 --dbc build/can/igvc_can.dbc &
./dbw/nodesd/nodesd.py --can-DBW_Enable &
./dbw/py-steering/py-steering.py &
./ctrl/velocity-control/ctrl.py &
# ctrl.py or vel.py in controller must just be updated to work with DBW_RawVelocityCommand
# change encoders to be  deltas from abs values
