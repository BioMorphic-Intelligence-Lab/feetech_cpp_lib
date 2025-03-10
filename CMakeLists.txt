cmake_minimum_required(VERSION 3.8)
project(feetech_lib)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
 
# define a library target called feetech_lib
add_library(feetech_lib src/feetech_lib.cpp)
ament_target_dependencies(feetech_lib)
target_include_directories(feetech_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/feetech_lib>
  $<INSTALL_INTERFACE:include>)

# add this line to exports the library
ament_export_targets(feetech_lib HAS_LIBRARY_TARGET)
 
# install the include/my_cpp_library directory to the install/include/my_cpp_library
install(
  DIRECTORY include/feetech_lib
  DESTINATION include
)
 
install(
  TARGETS feetech_lib
  EXPORT feetech_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_include_directories(include)
ament_export_dependencies(ament_cmake)
ament_export_libraries(feetech_lib)

ament_package()
