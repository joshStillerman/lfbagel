

add_executable(mds_pub_vl6180x mds_pub_vl6180x.cpp)
target_sources(mds_pub_vl6180x PRIVATE VL6180XSet.cpp Adafruit_VL6180X.cpp tiny_gpio.c)

add_executable(mds_sub_magnet mds_sub_pwm_out.cpp)
target_sources(mds_sub_magnet PRIVATE pca9685.c i2c.c ibt2.cpp Adafruit_ADS1X15.cpp tiny_gpio.c)

add_executable(mds_sub_magnet_pub_current_heights  mds_in_and_out.cpp)
target_sources(mds_sub_magnet_pub_current_heights  PRIVATE  VL6180XSet.cpp Adafruit_VL6180X.cpp 
                                        pca9685.c i2c.c ibt2.cpp Adafruit_ADS1X15.cpp 
                                        tiny_gpio.c)

add_executable(mds_out_cycle_test mds_out_cycle_test.cpp)
target_sources(mds_out_cycle_test PRIVATE pca9685.c i2c.c ibt2.cpp tiny_gpio.c)

#add_executable(mds_pub_mcc128 mcc128-pub.cpp)
#target_sources(mds_pub_mcc128 PRIVATE tiny_gpio.c)

add_library(VL6180XSet SHARED VL6180XSet.cpp Adafruit_VL6180X.cpp tiny_gpio.c)

add_library(ADS1X15 SHARED Adafruit_ADS1X15.cpp tiny_gpio.c i2c.c )

add_library(IBT2 SHARED ibt2.cpp tiny_gpio.c i2c.c )

target_link_libraries(mds_pub_vl6180x 
                        PRIVATE ev 
                        PRIVATE MDSplus::MdsObjectsCppShr 
                        )

target_link_libraries(mds_sub_magnet 
                        PRIVATE ev 
                        PRIVATE MDSplus::MdsObjectsCppShr
                        )

target_link_libraries(mds_sub_magnet_pub_current_heights 
                        PRIVATE ev 
                        PRIVATE MDSplus::MdsObjectsCppShr
                        )

target_link_libraries(mds_out_cycle_test
                        PRIVATE ev 
                        PRIVATE MDSplus::MdsObjectsCppShr
                        )
		#target_link_libraries(mds_pub_mcc128
		#                        PRIVATE ev 
		#                        PRIVATE MDSplus::MdsObjectsCppShr
		#                        PRIVATE daqhats)

		install(TARGETS mds_sub_magnet_pub_current_heights VL6180XSet ADS1X15 IBT2)

