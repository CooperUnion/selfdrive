#include "ember_app_desc.h"

const IN_DESC_SECTION ember_app_desc_v1_t ember_app_description = {
    .ember_magic = EMBER_MAGIC,
    .app_desc_version = EMBER_APP_DESC_VERSION,

    .node_identity = EMBER_NODE_IDENTITY,
};
