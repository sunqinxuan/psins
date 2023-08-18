PROJECT(INSdemo)
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

#set( CMAKE_BUILD_TYPE Release )
#set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )
#set(CMAKE_CXX_FLAGS "-fno-elide-constructors ${CMAKE_CXX_FLAGS}")
#add_compile_options(-fno-elide-constructors)
set(CMAKE_CXX_COMPILER "clang++")

ADD_SUBDIRECTORY(PSINSCore)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/PSINSCore)
link_directories(${PROJECT_SOURCE_DIR}/lib)
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)

add_executable(INSdemo main.cpp)
target_link_libraries(INSdemo psins)

