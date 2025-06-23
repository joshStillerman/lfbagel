# bagelpi
Code for the raspberry pi computers used by the levitated bagel

Builds programs
- mcc128-pub  
- mds_in_and_out  
- mds_out_cycle_test  
- mds_pub_vl6180x  
- mds_sub_magnet

Program executeables end up in build/src/

To build
```
mkdir build
cd build
cmake ..
make
```

files in ./etc/ can be copied to /etc/systemd/system to define services
