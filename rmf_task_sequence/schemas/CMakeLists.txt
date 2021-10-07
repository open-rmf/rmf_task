include(../cmake/generate_schema_header.cmake)

file(GLOB schema_files "*.schema.json")

foreach(schema_file ${schema_files})
  generate_schema_header(${schema_file})
endforeach()

install(
  DIRECTORY ${CMAKE_BINARY_DIR}/include/
  DESTINATION include
)
