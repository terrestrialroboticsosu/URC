project('serial_forwarder', 'cpp', default_options : ['cpp_std=c++17'])

# Include the ASIO headers
asio_inc = include_directories('asio-1.30.2/include')

executable('serial_forwarder',
           sources: ['main.cpp', 'SerialForwarder.cpp'],
           include_directories: [asio_inc])
