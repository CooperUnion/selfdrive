# ruff: noqa: F821

Import('env')


node = env.GetProjectOption('board_can_node')

network = env.File('../../can/network.yml')

build_dir = Dir(env['BUILD_DIR'])
gen_dir = build_dir.Dir(f'../opencan_generated/{node}')

env.Execute(Mkdir(gen_dir.path))
if env.Execute(f'opencan-cli codegen {network} {gen_dir} {node}'):
    print('OpenCAN error; stopping build.')
    exit(-1)

# Add gen_dir to the CPPPATH
env.Prepend(CPPPATH=[gen_dir])

# Get build sources from gen_dir
env.BuildSources(build_dir.path, gen_dir.path)
