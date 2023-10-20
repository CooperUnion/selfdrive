# Simple script to add a #define for EMBER_NODE_IDENTITY.

Import('env')

identity = env.GetProjectOption('board_node_identity', default=None)
if identity is None:
    identity = env.GetProjectOption('board_can_node', default=None)
    if identity is None:
        print(
            "Error: board_node_identity or board_can_node must be set in platformio.ini"
        )
        exit(-1)

    print(f"Using board_can_node as node identity: {identity}")

env.Append(CPPDEFINES=[("EMBER_NODE_IDENTITY", f'{identity}')])
