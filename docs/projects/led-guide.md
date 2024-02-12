# Creating your first node

## Repo Structure

At first the repository will seem a bit intimidating! There's plenty
going on here, but for the purposes of this guide, you don't need to
worry about most of it.

As a prequiste, make sure you've read through Jacob's
[getting started guide](https://github.com/CooperUnion/selfdrive#getting-started)
on setting up the repository. You should create your own branch in the
repo to work on this project.

We'll be starting off in the `selfdrive/dbw/node_fw/mod` directory the
directory, as shown below.

```bash
mod
├── blink
│   ├── blink.c
│   ├── blink.h
│   └── library.json
├── brake
│   ├── brake.c
│   ├── brake.h
│   └── library.json
├── ctrl
│   ├── ctrl.c
│   ├── ctrl.h
│   └── library.json
├── encoder
│   ├── encoder.c
│   ├── encoder.h
│   └── library.json
├── pb_mon
│   ├── library.json
│   ├── pb_mon.c
│   └── pb_mon.h
├── steer
│   ├── library.json
│   ├── steer.c
│   └── steer.h
├── sup
│   ├── library.json
│   ├── sup.c
│   └── sup.h
└── throttle
    ├── library.json
    ├── pedal.c
    ├── pedal.h
    ├── throttle.c
    └── throttle.h
```

Within this directory are all the `.c` and `.h` programs for each board.
This is where we'll be writing the firmware for our program!

## Creating `led.c` and `led.h`

Create a new `led` directory and within it, add `led.c`, `led.h` and a
`library.json` file.

Within `led.h` create a bare bones `.h` file to declare the contents of
what is being created in `led.c`. Next, create `led.c`, will require a
bit more explaining.

```c
#ifndef LED_H
#define LED_H

#endif
```

### Basic Structure

Each `.c` module in `mod` adheres to the following skeleton structure:

```c
// ######        DEFINES        ###### //

// ######     PRIVATE DATA      ###### //

// ######      PROTOTYPES       ###### //

// ######    RATE FUNCTIONS     ###### //

// ######   PRIVATE FUNCTIONS   ###### //

// ######   PUBLIC FUNCTIONS    ###### //

// ######        CAN TX         ###### //
```

*Note*: Prior to the `DEFINES` section, you need to `#include` the
libraries that will be used in your program!

In `.c`, `#include <file.h>` will search for libraries within standard
libraries that have already been associated with the platform.
`#include "file.h"` will typically search for files contained within the
same directory.

`// ######        DEFINES        ###### //`

Is where you will `#define` important constants/macros to be used within
your program.

For example, looking at `encoder.c` you'll see:
`#define ENCODER_MAX_TICKS 85` or `#define ENCODER0_CHAN_A 25`. These
are used because they are *read only* constants that are available
globally within the program. If you're interested, you should also take
a look at `#define` macros!

`// ######     PRIVATE DATA      ###### //`

Within the private data section, you declare global variables which will
be used within your program.

`// ######      PROTOTYPES       ###### //`

Next up we've got the prototypes section, which is essentially where
you'll pre-declare the non-rate based functions in your program (more on
rate functions later).

`// ######    RATE FUNCTIONS     ###### //`

Through our use of `FreeRTOS`, we've defined "skeleton" rate functions
which can run at 1Hz, 10Hz, 100Hz or 1kHz. These functions are generally
preceded by the creation of a struct:

```c
ember_rate_funcs_S module_rf = {
    .call_init   = MYMODULE.init,
    .call_1Hz    = MYMODULE.1Hz,
    .call_10Hz   = MYMODULE.10Hz,
    .call_100Hz  = MYMODULE.100Hz,
    .call_1kHz   = MYMODULE.1kHz,
};
```

*Note*: If you're not using all the rates available, you can leave them
out of the struct.

If the syntax for declaring this struct seems a bit challenging, take a
look at `typedef` and combining it with `structs` in C.

After creating this struct, we create respective functions for each
element defined within the struct.

```c
static void MYMODULE_init() {

}

static void MYMODULE_1Hz(){

}

// And so on....
```

`// ######   PRIVATE FUNCTIONS   ###### //`

Functions intended for use only within this `.c` file.

`// ######   PUBLIC FUNCTIONS    ###### //`

Functions intended for use both within and outside of this `.c` file. If
you have public functions, you'd put their prototypes in their
respective `.h` file so that they can be used outside of this `.c` file.

`// ######        CAN TX         ###### //`

We'll talk about what **CAN** is later, but just know that this is the
region where we populate CAN messages with information from our program.

### `led.c`

With that out of the way, we're now ready to add content to `led.c`! Try
looking through the firmware for the other modules and give this a shot
for yourself, the steps will be detailed below.

For starters, we need to `#include "led.h"`. Next we
`#include <driver/gpio.h>`. As you can tell by the `<>` brackets, we
didn't write the `driver/gpio.h` file ourselves, instead, it was
included with the **ESP-IDF**, a "development framework" which
essentially just contains a bunch of useful libraries that allow us to
interface with the hardware on the micronctoller!

```c
#include "led.h"

#include <driver/gpio.h>

#include "ember_common.h"
#include "ember_taskglue.h"
```

______________________________________________________________________

#### ESP-IDF

Whenever you need to perform some action on the ESP-32S3 (which is the
processor on our node board), you'll want to use the
[ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/release-v5.0/esp32s3/index.html)
as a reference).

To interface with gpio pins, which will allow us to toggle our LED on or
off, we need functions contained within `driver/gpio.h`. You should take
a look at the nicely commented code for `driver/gpio.h` on GitHub:
[GPIO driver](https://github.com/espressif/esp-idf/blob/release/v5.0/components/driver/include/driver/gpio.h)

______________________________________________________________________

### Populating the template

`// ######        DEFINES        ###### //`

For starters, let's pick a GPIO to use, make sure its available on the
node board (not being used for something else).

`#define LED_GPIO 18`

`// ######     PROTOTYPES        ###### //`

Add prototype functions so that we can use them without defining them in
the `module_rf` struct later.

```c
static void led_init();
static void led_1Hz();
```

`// ######     PRIVATE DATA      ###### //`

We want this GPIO to toggle between being on and off. Let's create a
`static int LED_STATUS` to store it's current state so that we can
toggle it on and off.

`// ######    RATE FUNCTIONS     ###### //`

We need to define the aforementioned `ember_rate_funcs_S` struct. Create
an init and a 1Hz function as follows:

```c
ember_rate_funcs_S module_rf = {
    .call_init = led_init,
    .call_1Hz = led_1Hz,
};
```

Then, create the two functions

```c
static void led_init()
{
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
}

static void led_1Hz()
{
    LED_STATUS = !LED_STATUS
    gpio_set_level(LED_GPIO, LED_STATUS);
}
```

All this program does is initialize our GPIO as an output pin within
`led_init` and then within our `led_1Hz` function we toggle its status
and set the gpio to that level, making the gpio blink.

Your file should look something like this:

```c
#include "led.h"

#include <driver/gpio.h>

#include "ember_common.h"
#include "ember_taskglue.h"

// ######        DEFINES        ###### //
#define LED_GPIO 18

// ######      PROTOTYPES       ###### //

static void led_init();
static void led_1Hz();

// ######     PRIVATE DATA      ###### //
static int LED_STATUS;

// ######    RATE FUNCTIONS     ###### //

ember_rate_funcs_S module_rf = {
    .call_init = led_init,
    .call_1Hz = led_1Hz,
};

static void led_init()
{
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
}

static void led_1Hz()
{
    LED_STATUS = !LED_STATUS
    gpio_set_level(LED_GPIO, LED_STATUS);
}

// ######   PRIVATE FUNCTIONS   ###### //

// ######   PUBLIC FUNCTIONS    ###### //
```

## Building the File

Now that we've written the code for our `led.c` and `led.h` files, it's
time to integrate it within our build system. This process is broken
down into the three main steps, detailed below:

- Adding to JSON
- Adding to Platform IO (fw)
- Adding to `can.yml`
- Adding to Platform IO (bl)

### JSON

JSON is a file formatting standard that allows for easy data
transportation. To create a node, you must create a respective
`library.json` file within the directory for your node, in the format
here:

```json
{
    "name": "node-name",
    "verson": "rolling",
    "description": "node description"
}
```

In our case,

```json
{
    "name": "cuber-led",
    "version": "rolling",
    "description": "simple node to turn on LED"
}
```

### Platform IO (fw)

Once you've finished making the `.json` file, it's time to add our new
node to a Platform IO configuration file. Platform IO is essentially a
build system that allows us to compile our firwmare for multiple
different microcontrollers. With Platform IO, we provide the details of
the device were using and it pulls the necessary files to compile the
firmware.

In `selfdrive/dbw/node_fw`, we can edit `platformio.ini` to add our new
node.

Insert the follow skeleton in alphabetical order:

```ini
[env:node]
board = ccmn_v2.0B
board_can_node = NODE_NAME
lib_deps =
    ${env.lib_deps}
    cuber-node
```

For us:

```ini
[env:led]
board = ccmn_v2.0B
board_can_node = LED
lib_deps =
    ${env.lib_deps}
    cuber-led
```

`ccm_v2.0B` : Is the version of DBW node board we're currently using.
The build system is different across boards because they have different
hardware.

### CAN

Each node has CAN messages associated with it that allow the node to
communicate with other nodes over a CAN network. We define CAN messages
and their respective nodes within the `can.yml` file in `selfdrive/can`

Further documentation on what a CAN message is will be provided soon, a
brief summary would be that CANbus is an automotative grade
communication standard used to communicate between peripherals on the
vehicle. CAN messages are sent (TX) and recieved (RX).

Within `can.yml` we have `message_templates` which define skeleton
structures for common CAN messages that will appear in multiple nodes.

Each CAN message is in `PascalCase`, with their respective signals in
`camelCase`. A CAN message can be though of one large message that gets
sent at a certain rate, the `cycletime`, and then a series of signals
that make up the message. Each signal has a `description` and a `width`,
with many more options also existing, but you'll discover those when you
need them.

Beneath the `message_templates` we have the `nodes` section, which
defines the messages for the nodes on the vehicle. Each node has a `rx`
(CAN messages it recieves), `tx` (CAN messages it transmits) and a
`messages` sections which populates the details of each message. Each
message within the `can.yml` file has a unique identifier associated
with it. Additionally, each entry is placed within alphabetical order,
so when adding new nodes/messages be sure to provide a unique ID and
maintain the order of entries. Also note that YAML files are whitespace
sensitive, meaning you need to adhere to the spacing format (like
python).

We now add our `LED` node to the `can.yml`

```yml
  - LED:
      rx:
        - DBW_ESTOP
        - UPD_UpdateControl_LED  # bootloader
      messages:
        - NodeInfo:
            from_template: DBWNodeInfo
            id: 0xD7

        - NodeStatus:
            from_template: DBWNodeStatus
            id: 0xE7

  - LEDBL:
      rx:
        - UPD_IsoTpTx_LED
        - UPD_UpdateControl_LED
      messages:
        - IsoTpTx:
            id: 0x601
        - Status:
            from_template: BlStatus
            id: 0x602
```

Later on in the `can.yml` file, we need to also update the following
section:

```yml
  - UPD:
      rx:
        - LED_NodeStatus
        - LEDBL_IsoTpTx
        - LEDBL_Status

      messages:
        - IsoTpTx_LED:
            id: 0x533
        - UpdateControl_LED:
            from_template: UpdateControl
            id: 0x5A3
```

### Platform IO (bl)

We also need to update our respective `platformio.ini` file within
`selfdrive/dbw/ember_bl` to include this node:

```ini
[env:led]
board = ccmn_v2.0B
board_can_node = LEDBL
board_node_identity = LED
```

With that done, we're now ready to upload the firmware onto the node
boards.

In `selfdrive` type `scons`. If you've properly set up Nix, the
following output should appear: scons: Reading SConscript files ...

```
 So you want to build a car?

 You can specify targets after `scons`, like:

 `scons clean`                         Clean (remove) build/ directory
 `scons dbc`                           Build igvc_can.dbc
 `scons ember_bl`                      Build dbw/ember_bl
 `scons ember_bl --blpioflags=FLAGS`   Run pio for ember_bl with FLAGS, e.g. `scons ember_bl --blpioflags="run -e blink1.1"`
 `scons node_fw`                       Build dbw/node_fw
 `scons node_fw --pioflags=FLAGS`      Run pio for node_fw with FLAGS, e.g. `scons node_fw --pioflags="run -e blink1.1"`


 Note: try these helpful aliases (if you have `direnv`):

 `fwpio`    Equivalent to `pio`, but specifically for dbw/node_fw, can
            be used anywhere in the repo, and uses scons. This is the
            recommended way to use PlatformIO. For example:

                $ fwpio run -e blink1.1
```

For starters, you'll want to run `scons dbc`, `scons ember_bl`, and
`scons node_fw`. They'll set up the CAN database, the bootloader to
upload your firmware to the node boards, and lastly all the nodes
currently specified within `mod` and `platformio.ini`.

To run the firmware, make sure you've allowed direnv (`direnv allow`).

Then run, `fwpio -e run led`

## USB Passthrough

There are two ways of uploading firmware to the node board, either
through USB or CAN. For this project we'll be doing it over USB. For
starters, if you're not on a native UNIX machine you'll need to ensure
that you have enabled USB passthrough. If you're using `WSL`, check out:
[Connecting USB Devices using usbipd](https://learn.microsoft.com/en-us/windows/wsl/connect-usb).

Once you plug your node board into your machine, it should show up as a
serial device within your machine. On UNIX, you can find it within
`/dev/tty` and it will generally be in the form of `ttyACM`. If you
can't seem to find it, try printing out all the contents `/dev/tty*` and
using `grep` to inform your search.

If the device pops up, you can upload your firmware to board using

Once that's finished building, you're ready to run
`fwpio run -e led -t upload --upload-port /dev/ttyACM0`

Which will build the firmware for your board and attempt to upload it to
the serial device specified at the end.

If the command succeeds and your code works, you should see your LED
blink!
