cmake_minimum_required(VERSION 3.0)

project(SoftPosit)
set(try softposit)
FIND_PACKAGE(OpenCV REQUIRED)
include_directories(${JSONCPP_LIBRARIES})
add_executable(${try} main.cpp)
# target_link_libraries(${try} softposit)

set(lib_DIR /usr/lib/x84_64-linux-gnu)
link_directories(${lib_DIR})

find_package(PCL 1.10 REQUIRED COMPONENTS common io)    # 说明工程需要包含的PCL版本以及对应的模块，REQUIRED表示如果没有这些模块，就报错。FIND_PACKAGE(PCL 1.10 REQUIRED)表示包含所有组件
FIND_PACKAGE(PCL 1.10 REQUIRED COMPONENTS common io visualization ) # you can add FIND_PACKAGE()
include_directories("/usr/include/pcl-1.10" "/usr/include/vtk-7.1")
link_directories(${PCL_LIBRARY_DIRS}) # 当找到PCL库时，PCL_LIBRARY_DIRS会被自动设置

target_link_libraries(${try} libjsoncpp.a)
target_link_libraries(${try}  armadillo)
target_link_libraries(${try} ${OpenCV_LIBS})
target_link_libraries(${try} ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${VTK_LIBRARY_DIRS} ${PCL_VISUALIZATION_LIBRARIES}) 
