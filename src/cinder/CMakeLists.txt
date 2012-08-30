add_subdirectory(app)
add_library(cinder)
target_link_libraries(cinder app)