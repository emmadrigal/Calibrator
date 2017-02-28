cmake_minimum_required(VERSION 2.8)

project( calibrator )

add_executable( Calibrator calibrator.cpp )

find_package( OpenCV REQUIRED )
find_package( Boost COMPONENTS program_options REQUIRED )


target_link_libraries( Calibrator ${OpenCV_LIBS} ${Boost_LIBRARIES})
