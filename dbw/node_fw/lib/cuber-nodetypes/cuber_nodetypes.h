#ifndef CUBER_NODETYPES_H
#define CUBER_NODETYPES_H

#define MODULE_TYPE_SCHEMA 0

// This is used in various places and must be synchronized with CAN definitions.
enum cuber_node_types {
    NODE_UNDEF = 0,
    NODE_BLINK = 1,
    NODE_THROTTLE = 2,
    NODE_BRAKE = 3,
    NODE_ENCODER = 4,
    NODE_CTRL = 5,
    NODE_PB_MON = 6,
};

extern const enum cuber_node_types CUBER_NODE_IDENTITY;

#endif
