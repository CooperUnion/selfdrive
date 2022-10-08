// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * brake.h -- brake module
 *
 * Copyright (C) 2022  Dan Mezhiborsky <daniel.mezhiborsky@cooper.edu>
 * Copyright (C) 2022  Jacob Koziej <jacobkoziej@gmail.com>
 */

#ifndef BRAKE_H
#define BRAKE_H

#include "io/can.h"

extern struct CAN_dbwNode_Vel_Cmd_t CAN_Vel_Cmd;
extern can_incoming_t can_Vel_Cmd_cfg;

#endif
