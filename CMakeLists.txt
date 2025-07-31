cmake_minimum_required(VERSION 3.10)
project(self_define)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

# 包含头文件目录
include_directories(
    ${CMAKE_SOURCE_DIR}/src
    /home/ubuntu/桌面/FT_servo/FTServo_Linux/src
)

# 设置源文件
set(SOURCES
    main.cpp
    servo.cpp
)

set(HEADERS
    servo.h
)

# 添加可执行文件
add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})

# 显式链接静态库的路径（注意，这里不是 link_directories）
target_link_libraries(${PROJECT_NAME}
    /home/ubuntu/桌面/FT_servo/FTServo_Linux/src/libSCServo.a
    pthread
)

