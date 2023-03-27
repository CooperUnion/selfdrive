# Simple script to add a #define for EMBER_NODE_IDENTITY.

Import('env')

identity = env.GetProjectOption('board_node_identity')
env.Append(CPPDEFINES=[("EMBER_NODE_IDENTITY", f'\\\"{identity}\\\"')])
