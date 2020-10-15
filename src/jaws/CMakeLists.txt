add_library(jaws Gripper.cpp rs232.cpp)

add_executable(GripperMain GripperMain.cpp)
target_link_libraries(GripperMain jaws)
