#ifndef MODULE_TYPES_H

#define MODULE_TYPE_SCHEMA 0

// This is used in various places and must be synchronized with CAN definitions.
enum firmware_module_types {
    MOD_UNDEF = 0,
    MOD_BLINK = 1,
    MOD_THROTTLE = 2,
    MOD_BRAKE = 3,
    MOD_ENCODER = 4,
    MOD_REAR_ENCODER = 5,
    MOD_PB_MON = 6,
};

extern const enum firmware_module_types FIRMWARE_MODULE_IDENTITY;

#endif
