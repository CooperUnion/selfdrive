#!/bin/bash

make
cantools generate_c_source igvc_can.dbc --database-name CAN --use-float
mv CAN.c can_gen.c
mv CAN.h can_gen.h

sed -i 's\CAN_H\CAN_GEN_H\g' can_gen.h  
sed -i 's\CAN.h\can_gen.h\g' can_gen.c

cp can_gen.* ../dbw/node_fw/src/io/
