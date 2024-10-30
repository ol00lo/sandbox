   cmake_minimum_required(VERSION 3.10)
   project(a)

   set(BOOST_ROOT ${CMAKE_SOURCE_DIR}/../../boost_1_86_0)
   find_package(Boost REQUIRED COMPONENTS system thread)

   if(Boost_FOUND)
       message(STATUS "Boost found")
   else()
       message(FATAL "Boost not found")
   endif()

   add_executable(a a.cpp http_response.cpp http_response.hpp)
   target_link_libraries(a Boost::system Boost::thread)