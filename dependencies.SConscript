# Dependencies
# Here, we handle individual dependencies as needed.

Import("env")

# venv ----------------------------------------------------
# Check that direnv is installed.
if env.Execute('which direnv') != 0:
  print("Error: Couldn't find direnv. Please install it.")
  exit(-1)

env['VENV_DIR'] = env.Dir('$REPO_ROOT/deps/venv')

# Make the venv
env.Command(
  env['VENV_DIR'],
  [],
  'python3 -m venv $TARGET'
)

# Add the venv/bin folder to PATH
env.PrependENVPath('PATH', env['VENV_DIR'].Dir('bin'))
# Set VIRTUAL_ENV
env['ENV']['VIRTUAL_ENV'] = env['VENV_DIR']

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
env['RUST_VERSION'] = '1.68.2'
RUST_HOME           = env.Dir('$REPO_ROOT/deps/rust/$RUST_VERSION')
RUST_TOOLS_PATH     = RUST_HOME.Dir('bin')
env['CARGO']        = RUST_TOOLS_PATH.File('cargo')

env['ENV']['CARGO_HOME']  = RUST_HOME
env['ENV']['RUSTUP_HOME'] = RUST_HOME

[rust_install_builder] = env.Command(
  env['CARGO'], # picking cargo as the target file
  [],
  'curl https://sh.rustup.rs -sSf | sh -s -- -y --no-modify-path --default-toolchain $RUST_VERSION'
)

env.PrependENVPath('PATH', RUST_TOOLS_PATH)
env.Alias('deps-rust', rust_install_builder)
# ---------------------------------------------------------


# OpenCAN -------------------------------------------------
OPENCAN_VERSION     = 'b014266'
env['OPENCAN_CLI']  = env.File('$REPO_ROOT/build/cargo/bin/opencan-cli')

[opencan_cli_builder] = env.Command(
    env['OPENCAN_CLI'],
    env['CARGO'],
    f'$CARGO install --root $REPO_ROOT/build/cargo --locked --git https://github.com/opencan/opencan --rev {OPENCAN_VERSION}'
)

env.Alias('opencan_cli', opencan_cli_builder)
# ---------------------------------------------------------
