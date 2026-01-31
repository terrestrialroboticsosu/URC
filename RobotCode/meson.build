project(
  'urc',
  'cpp',
  version: '1.0.0',
  license: 'MIT',
  default_options: [],
)

# Add global defines
add_global_arguments('-DSOME_TOKEN=value', language: 'cpp')

# Include directory for project headers
inc = include_directories('include')

# List all source files under src/ (excluding robotArm.cpp)
proj_sources = files(
  'src/main.cpp',
  'src/driverStation.cpp',
  'src/robotActuation.cpp',
  'src/robotControl.cpp',
  'src/robotSerial.cpp',
  'src/robotState.cpp',
  'src/util.cpp'
)

# ASIO subproject
asio = subproject('asio')
asio_dep = asio.get_variable('asio_dep')

# Link arguments for Windows only
link_args = ''
if host_machine.system() == 'windows'
  link_args += '-lwsock32'
endif

# Build executable
executable(
  'urcbot',
  proj_sources,
  include_directories: inc,
  dependencies: [asio_dep],
  link_args: link_args
)
