include(../cmake/generate_schema_header.cmake)

# Current way to get rmf_api msgs pkg path, and then shared schemas
# Note: better way is to use ament_cmake fn
set(rmf_api_msgs_path "")
execute_process(
  OUTPUT_VARIABLE rmf_api_msgs_path
  COMMAND ros2 pkg prefix rmf_api_msgs
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
set(rmf_api_schemas_path ${rmf_api_msgs_path}/share/rmf_api_msgs/schemas)
message("API schemas path: [" ${rmf_api_schemas_path} "]")

message( ${rmf_api_schemas_path}/*.schema.json)

file(GLOB schema_files ${rmf_api_schemas_path}/*.schema.json)

foreach(schema_file ${schema_files})
  generate_schema_header(${schema_file})
  message(" - Generating schema header for: " ${schema_file})
endforeach()

install(
  DIRECTORY ${CMAKE_BINARY_DIR}/include/
  DESTINATION include
)
