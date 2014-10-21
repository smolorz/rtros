cmake_minimum_required(VERSION 2.8.3)
project(rtros)

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rtros
)

include_directories(include ${catkin_INCLUDE_DIRS} ${RACKRTROS_INCLUDE_DIR} ${XENOMAI_INCLUDE_DIR})

add_library(rtros src/publisher.cpp src/init.cpp src/subscriber.cpp)

target_link_libraries(rtros ${catkin_LIBRARIES})

install(TARGETS rtros LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")
