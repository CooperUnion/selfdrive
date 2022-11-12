# Ember Bootloader Overview

The Ember bootloader is an embedded bootloader for CAN connected
microcontrollers. Its job is to wait for an update initiation CAN message upon
boot and either boot into the application firmware or accept a new application
firmware image over CAN.

The firmware image is transferred over ISO-TP.

The bootloader reporting states are as follows:

STATE                         DESCRIPTION                               NEXT STATE (all states can go to BL_FAULT)
BL_WAIT_FOR_TRIGGER...........WAIT FOR TRIGGER MESSAGE..................BL_BOOT_INTO_FW or BL_ISOTP_NEGOTIATE
BL_BOOT_INTO_FW...............BOOTING INTO APPLICATION..................(boot into application)
BL_ISOTP_NEGOTIATE............WAIT FOR ISOTP TRANSFER TO BEGIN..........BL_ISOTP_TRANSFER
BL_ISOTP_TRANSFER.............ACTIVE ISOTP TRANSFER.....................BL_WRITEBACK
BL_WRITEBACK..................WRITING BUFFER TO FLASH...................BL_ISOTP_TRANSFER or BL_TRY_COMMIT_UPDATE
BL_TRY_COMMIT_UPDATE..........TRY TO FINALIZE UPDATE....................BL_BOOT_INTO_FW OR BL_FAULT
BL_FAULT......................BOOTLOADER FAULT..........................(reset SoC)

# To apply a firmware update:

1. Send the trigger message and wait for BL_ISOTP_NEGOTIATE.
2. Send the image over ISO-TP.
   a. ISO-TP block size is 4096 bytes and timing is 1 ms/frame.
   b. After each block, the state will change to BL_WRITEBACK as the bootloader
      writes the block to flash. Then, an ISO-TP flow control frame will be
      sent and the state will change back to BL_ISOTP_TRANSFER.
   c. Once the ISO-TP transfer is complete, expect BL_TRY_COMMIT_UPDATE.
3. Expect either BL_BOOT_INTO_FW or BL_FAULT.

# For identity:

1. The Ember bootloader is meant to be built and flashed with a particular set
   of CAN message IDs that define its identity.
2. To change the identity of a node, flash the corresponding bootloader
   over USB/JTAG.

There is no method to dynamically change the node identity. This is a
deliberate design choice and is meant to minimize the possibility of
incorrect identification.

The Ember bootloader expects the following to be preprocessor defined at
bootloader build time:

```c
#define EMBER_BL_TRIGGER_MSG_ID    (some CAN message ID) // rx
#define EMBER_BL_REPORTING_MSG_ID  (some CAN message ID) // tx

#define EMBER_BL_ISOTP_SEND_MSG_ID (some CAN message ID) // tx
#define EMBER_BL_ISOTP_DEST_MSG_ID (some CAN message ID) // rx
```

