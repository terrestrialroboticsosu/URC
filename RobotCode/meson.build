project(
  'urc',
  'cpp',
  version: '1.0.0',
  license: 'MIT',
  default_options: [],
)

add_global_arguments('-DSOME_TOKEN=value', language: 'cpp')

# Combine all include directories into a single variable
inc = include_directories('include', 'libs/orocos-kdl/include')

proj_sources = [
  'src/main.cpp',
  'src/driverStation.cpp',
  'src/robotActuation.cpp',
  'src/robotControl.cpp',
  'src/robotState.cpp',
  'src/robotSerial.cpp',
  'src/util.cpp',
  'src/robotArm.cpp'
]

asio = subproject('asio')
asio_dep = asio.get_variable('asio_dep')

# Since the include path is handled above, this dependency now only needs the library file.
cpp = meson.get_compiler('cpp')
orocos_kdl_lib = cpp.find_library('orocos-kdl', dirs: meson.current_source_dir() + '/libs/orocos-kdl/lib')
orocos_kdl_dep = declare_dependency(
  dependencies: [orocos_kdl_lib]
)

link_args = ''
if build_machine.system() == 'windows'
  link_args += '-lwsock32' # Required by ASIO
endif

executable(
  'urcbot',
  proj_sources,
  # Use the combined include directories here
  include_directories: inc,
  dependencies: [asio_dep, orocos_kdl_dep],
  link_args: link_args
)