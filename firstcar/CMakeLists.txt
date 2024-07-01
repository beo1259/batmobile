cmake_minimum_required(VERSION 3.1)
project(Batmobile)

set(CMAKE_CXX_STANDARD 11)

find_library(PIGPIO_LIBRARY NAMES pigpio PATHS /usr/lib)
find_library(PIGPIOIF2_LIBRARY NAMES pigpiod_if2 PATHS /usr/lib)
find_library(PIGPIOD_IF2_LIBRARY NAMES pigpiod_if2 PATHS /usr/lib)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBEVDEV REQUIRED libevdev)

include_directories(/usr)
include_directories(/usr/include)
include_directories(/usr/include ${LIBEVDEV_INCLUDE_DIRS})

add_executable(main src/main.cpp)

target_link_libraries(main ${PIGPIO_LIBRARY} ${PIGPIOIF2_LIBRARY} ${PIGPIOD_IF2_LIBRARY} ${HIDAPI_LIBRARIES} ${LIBEVDEV_LIBRARIES})

