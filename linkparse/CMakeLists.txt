   cmake_minimum_required(VERSION 3.10)
   project(a)

   set(BOOST_ROOT ${CMAKE_SOURCE_DIR}/../../boost_1_86_0)
   find_package(Boost REQUIRED COMPONENTS system thread)

   set(SOURCE_FILE a.cpp)

   add_executable(a ${SOURCE_FILE})
   target_link_libraries(a Boost::system Boost::thread)