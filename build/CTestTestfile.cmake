# CMake generated Testfile for 
# Source directory: /home/righif5/progetto2024-pippo-pelaz
# Build directory: /home/righif5/progetto2024-pippo-pelaz/build
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
if(CTEST_CONFIGURATION_TYPE MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
  add_test([=[boids.t]=] "/home/righif5/progetto2024-pippo-pelaz/build/Debug/boids.t")
  set_tests_properties([=[boids.t]=] PROPERTIES  _BACKTRACE_TRIPLES "/home/righif5/progetto2024-pippo-pelaz/CMakeLists.txt;53;add_test;/home/righif5/progetto2024-pippo-pelaz/CMakeLists.txt;0;")
elseif(CTEST_CONFIGURATION_TYPE MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
  add_test([=[boids.t]=] "/home/righif5/progetto2024-pippo-pelaz/build/Release/boids.t")
  set_tests_properties([=[boids.t]=] PROPERTIES  _BACKTRACE_TRIPLES "/home/righif5/progetto2024-pippo-pelaz/CMakeLists.txt;53;add_test;/home/righif5/progetto2024-pippo-pelaz/CMakeLists.txt;0;")
elseif(CTEST_CONFIGURATION_TYPE MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
  add_test([=[boids.t]=] "/home/righif5/progetto2024-pippo-pelaz/build/RelWithDebInfo/boids.t")
  set_tests_properties([=[boids.t]=] PROPERTIES  _BACKTRACE_TRIPLES "/home/righif5/progetto2024-pippo-pelaz/CMakeLists.txt;53;add_test;/home/righif5/progetto2024-pippo-pelaz/CMakeLists.txt;0;")
else()
  add_test([=[boids.t]=] NOT_AVAILABLE)
endif()
