# Dependencies
# Here, we handle individual dependencies as needed.

Import("env")

# VERSIONS ------------------------------------------------
env['RUST_VERSION']    = '1.68.2'
env['OPENCAN_VERSION'] = 'b014266'
# ---------------------------------------------------------

# venv & pip ----------------------------------------------
env['VENV_DIR'] = env.Dir('venv')

# Make the venv
env.Command(
  env['VENV_DIR'],
  [],
  'python3 -m venv $TARGET'
)

# Add the venv/bin folder to PATH
env.PrependENVPath('PATH', env['VENV_DIR'].Dir('bin').abspath)
# Set VIRTUAL_ENV
env['ENV']['VIRTUAL_ENV'] = env['VENV_DIR'].abspath

# pip packages
REQUIREMENTS = env.File('$REPO_ROOT/requirements.txt')

[pip_deps_builder] = env.Command(
  env['VENV_DIR'].File('.requirements-installed'),
  REQUIREMENTS,
  [
    '$VENV_DIR/bin/pip3 install -r $SOURCE',
    'touch $TARGET'
  ]
)

env.Alias('deps-pip', pip_deps_builder)
env['PIP_PACKAGES'] = pip_deps_builder
# ---------------------------------------------------------


# Rust ----------------------------------------------------
RUST_HOME           = env.Dir('rust/$RUST_VERSION')
RUST_TOOLS_PATH     = RUST_HOME.Dir('bin')
env['CARGO']        = RUST_TOOLS_PATH.File('cargo')

env['ENV']['CARGO_HOME']  = RUST_HOME
env['ENV']['RUSTUP_HOME'] = RUST_HOME

# https://blog.rust-lang.org/inside-rust/2023/01/30/cargo-sparse-protocol.html
env['ENV']['CARGO_REGISTRIES_CRATES_IO_PROTOCOL'] = 'sparse'

[rust_install_builder] = env.Command(
  env['CARGO'], # picking cargo as the target file
  [],
  'curl https://sh.rustup.rs -sSf | ' \
  'sh -s -- -y --no-modify-path --default-toolchain $RUST_VERSION'
)

env.PrependENVPath('PATH', RUST_TOOLS_PATH.abspath)
env.Alias('deps-rust', rust_install_builder)
# ---------------------------------------------------------


# OpenCAN -------------------------------------------------
env['OPENCAN_CLI'] = env['ENV']['CARGO_HOME'].File('bin/opencan-cli')

# Note that we have $CARGO_HOME set above. Cargo will install packages there.
[opencan_cli_builder] = env.Command(
    env['OPENCAN_CLI'],
    env['CARGO'],
    '$CARGO install --locked ' \
    '--git https://github.com/opencan/opencan --rev $OPENCAN_VERSION'
)

# Symlink to current version for platformio
# Since the path of the actual executable like deps/rust/1.68.2/bin,
# it's not easily predictable from opencan-pio.py, and we want pio
# to still work even when not invoked through scons and/or direnv.
#
# We make this symlink so opencan-pio.py has a reliable thing to use
# as the opencan-cli executable.
OPENCAN_SYMLINK = env.File('opencan-cli')
[opencan_symlink_builder] = env.Command(
    OPENCAN_SYMLINK,
    env['OPENCAN_CLI'],
    f'ln -sf $OPENCAN_CLI.abspath {OPENCAN_SYMLINK.abspath}'
)

env.Alias('deps-opencan', [opencan_cli_builder, opencan_symlink_builder])
# ---------------------------------------------------------
