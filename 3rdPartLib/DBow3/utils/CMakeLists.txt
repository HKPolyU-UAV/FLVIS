cmake_minimum_required(VERSION 3.5)

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src )
LINK_LIBRARIES(${PROJECT_NAME})

add_executable(voc_trans bin_vocabulary.cc)


