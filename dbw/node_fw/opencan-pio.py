Import("env")

node        = env.GetProjectOption("board_can_node")
yml         = env.File("../../can/can.yml")
opencan     = env.File("../../build/cargo/bin/opencan-cli")
build_dir   = Dir(env['BUILD_DIR'])
gen_dir     = build_dir.Dir(f"../opencan_generated/{node}")

# These Execute calls call opencan-cli every time.
# They could be replaced with a proper Command, but it was tricky getting
# PlatformIO to reliably execute them. You could ask them on GitHub or similar.
env.Execute(Mkdir(gen_dir.path))

if 0 != env.Execute(f'{opencan} codegen {yml} {gen_dir} {node}'):
    print("OpenCAN error, stopping build.")
    exit(-1)

# Add gen_dir to the CPPPATH
env.Prepend(CPPPATH=[gen_dir])

# Get build sources from gen_dir
env.BuildSources(build_dir.path, gen_dir.path)
