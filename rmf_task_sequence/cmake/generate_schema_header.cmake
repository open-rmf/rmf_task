#################################################
# generate_schema_header(<file_name>)
#
# This function takes a schema file and generates a C++ header
# file that hardcodes the schema into it as a const string.
function(generate_schema_header file_name)
  get_filename_component(schema_name ${file_name} NAME_WE)
  string(TOUPPER ${schema_name} upper_schema_name)
  configure_file(
    ../templates/schemas_template.hpp.in
    )
endfunction()
