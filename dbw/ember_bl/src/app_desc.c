#include "ember_app_desc.h"

// stringification macros
#define XSTR(s) STR(s)
#define STR(s)	#s

const IN_DESC_SECTION ember_app_desc_v1_t ember_app_description = {
    .ember_magic      = EMBER_MAGIC,
    .app_desc_version = EMBER_APP_DESC_VERSION,

    .node_identity = XSTR(EMBER_NODE_IDENTITY),
};
