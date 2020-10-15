include_directories(./camera)
include_directories(./util)
include_directories(./robot)
include_directories(./jaws)

add_executable(01-PhotoCapture 01-PhotoCapture.cpp)
target_link_libraries(01-PhotoCapture ${OpenCV_LIBRARIES} kinect_camera util)

add_executable(01-PhotoCapture2 01-PhotoCapture2.cpp)
target_link_libraries(01-PhotoCapture2 ${OpenCV_LIBRARIES} kinect_camera util)

add_executable(02-TemplateCloudFilter 02-TemplateCloudFilter.cpp)
target_link_libraries(02-TemplateCloudFilter ${PCL_LIBRARIES})

add_executable(03-TemplateRtMaker 03-TemplateRtMaker.cpp)
target_link_libraries(03-TemplateRtMaker ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

add_executable(04-TestGrabTemplate 04-TestGrabTemplate.cpp)
target_link_libraries(04-TestGrabTemplate ${OpenCV_LIBRARIES} aubo jaws)

add_executable(05-TemplateAlignment 05-TemplateAlignment.cpp)
target_link_libraries(05-TemplateAlignment ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} util )

add_executable(06-GrabTarget 06-GrabTarget.cpp)
target_link_libraries(06-GrabTarget ${OpenCV_LIBRARIES} aubo jaws)