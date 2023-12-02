add_library(path_finding_task Map.cpp)

message("Path_finding current source dir = ${CMAKE_CURRENT_SOURCE_DIR}")
 
target_include_directories( path_finding_task PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)