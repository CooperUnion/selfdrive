#!/bin/bash

candump -ta can0 | cantools decode "igvc_can.dbc" -s > "test.log" &

exec $@

# cat test.log | cantools plot -o plot.png igvc_can.dbc "Encoder*" , "ThrottleCmd" --style seaborn
