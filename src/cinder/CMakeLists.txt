add_subdirectory(app)
add_library(cinder Camera.cpp)
target_link_libraries(cinder app)