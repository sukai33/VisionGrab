add_library(util CloudMaker.h CloudMaker.cpp CoordinateUtil.h)
target_link_libraries(util ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})