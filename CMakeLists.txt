project(ocv_tests)

set(CMAKE_CXX_FLAGS "-Wall -Wextra -ansi -pedantic")

#add_executable(ocv_tests main.cpp)
#target_link_libraries(ocv_tests
#        opencv_core
#        opencv_highgui
#)

#add_executable(color_segment color_segment.cpp)
#target_link_libraries(color_segment
#        opencv_core
#        opencv_highgui
#)

#add_executable(bottracker bottracking.cpp bottracker.cpp utils.cpp)
#target_link_libraries(bottracker
#        opencv_core
#        opencv_highgui
#)

add_executable(tracktest
        utils.cpp
        ball_sample.cpp
        bot_sample.cpp
        field_sample.cpp
        bottracker.cpp
        tracktest.cpp
)

target_link_libraries(tracktest
        opencv_core
        opencv_highgui
        rt
	opencv_gpu
	opencv_imgproc
)

#add_executable(pruebas pruebas.cpp utils.cpp)
#target_link_libraries(pruebas
#        opencv_core
#        opencv_highgui
#        gtest
#)
