cmake_minimum_required(VERSION 3.5.0)

project(rmf_task_sequence VERSION 2.0.0)

set(CMAKE_EXPORT_COMPILE_COMMANDS on)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  # Use the Release build type by default if the user has not specified one
  set(CMAKE_BUILD_TYPE Release)
endif()

include(GNUInstallDirs)

find_package(rmf_api_msgs REQUIRED)
find_package(rmf_task REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(nlohmann_json_schema_validator_vendor REQUIRED)

find_package(ament_cmake_catch2 QUIET)
find_package(ament_cmake_uncrustify QUIET)

# ===== RMF Task Sequence Library
file(GLOB_RECURSE lib_srcs
  "src/rmf_task_sequence/*.cpp"
)

add_library(rmf_task_sequence SHARED
  ${lib_srcs}
)

target_link_libraries(rmf_task_sequence
  PUBLIC
    rmf_task::rmf_task
    nlohmann_json::nlohmann_json
    nlohmann_json_schema_validator
)

target_include_directories(rmf_task_sequence
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/rmf_api_generate_schema_headers/include> # for auto-generated schema headers
)

if(BUILD_TESTING AND ament_cmake_catch2_FOUND AND ament_cmake_uncrustify_FOUND)
  file(GLOB_RECURSE unit_test_srcs "test/*.cpp")
  ament_add_catch2(
    test_rmf_task_sequence test/main.cpp ${unit_test_srcs}
    TIMEOUT 300)
  target_link_libraries(test_rmf_task_sequence
      rmf_task_sequence
  )

  target_include_directories(test_rmf_task_sequence
    PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/>
  )

  find_file(uncrustify_config_file
    NAMES "rmf_code_style.cfg"
    PATHS "${rmf_utils_DIR}/../../../share/rmf_utils/")

  ament_uncrustify(
    ARGN include src test
    CONFIG_FILE ${uncrustify_config_file}
    MAX_LINE_LENGTH 80
  )
endif()

# Generate the schema headers
rmf_api_generate_schema_headers(
  PACKAGE rmf_task_sequence
  SCHEMAS_DIR ${CMAKE_CURRENT_LIST_DIR}/schemas
)


# Create cmake config files
include(CMakePackageConfigHelpers)

set(INSTALL_CONFIG_DIR "${CMAKE_INSTALL_LIBDIR}/rmf_task_sequence/cmake")
set(PACKAGE_CONFIG_VERSION_FILE "${CMAKE_CURRENT_BINARY_DIR}/rmf_task_sequence-config-version.cmake")
set(PACKAGE_CONFIG_FILE "${CMAKE_CURRENT_BINARY_DIR}/rmf_task_sequence-config.cmake")

configure_package_config_file(
  "${CMAKE_CURRENT_LIST_DIR}/cmake/rmf_task_sequence-config.cmake.in"
  ${PACKAGE_CONFIG_FILE}
  INSTALL_DESTINATION ${INSTALL_CONFIG_DIR}
)

write_basic_package_version_file(
  ${PACKAGE_CONFIG_VERSION_FILE}
  COMPATIBILITY ExactVersion
)

install(
  TARGETS rmf_task_sequence
  EXPORT  rmf_task_sequence-targets
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(
  DIRECTORY include/rmf_task_sequence
  DESTINATION include
)

install(
  FILES
    ${PACKAGE_CONFIG_VERSION_FILE}
    ${PACKAGE_CONFIG_FILE}
  DESTINATION ${INSTALL_CONFIG_DIR}
)

install(
  EXPORT rmf_task_sequence-targets
  FILE rmf_task_sequence-targets.cmake
  NAMESPACE rmf_task_sequence::
  DESTINATION ${INSTALL_CONFIG_DIR}
)

export(
  EXPORT rmf_task_sequence-targets
  FILE ${CMAKE_CURRENT_BINARY_DIR}/rmf_task_sequence-targets.cmake
  NAMESPACE rmf_task_sequence::
)

# Export ament index path
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/share/ament_index/resource_index/packages/${PROJECT_NAME} "")
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/share/ament_index/resource_index/packages/${PROJECT_NAME} DESTINATION share/ament_index/resource_index/packages)

file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/ament_prefix_path.dsv "prepend-non-duplicate;AMENT_PREFIX_PATH;")
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/ament_prefix_path.dsv DESTINATION share/${PROJECT_NAME}/hook)
