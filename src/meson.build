inout_srcs = [
    'Adafruit_VL6180X.cpp',
    'VL6180XSet.cpp',
    'Adafruit_ADS1X15.cpp',
    'pca9685.c',
    'ibt2.cpp',
    'i2c.c',
    'tiny_gpio.c',
    'mds_in_and_out.cpp'
]

out_srcs = [
    'pca9685.c',
    'ibt2.cpp',
    'i2c.c',
    'tiny_gpio.c',
    'mds_out_cycle_test.cpp'
]

if get_option('buildtype') == 'debug'
    add_project_arguments('-DDEBUG', language : ['c', 'c++'])
endif

executable('mds_in_and_out', inout_srcs, 
    include_directories: [mdsplus_inc], 
    dependencies: [mdsplusobj_lib, libev] )

executable('mds_out_cycle_test', inout_srcs, 
    include_directories: [mdsplus_inc], 
    dependencies: [mdsplusobj_lib, libev] )
