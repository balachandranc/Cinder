include_directories(include)
add_executable(EaseGallery src/EaseGalleryApp.cpp)
target_link_libraries(EaseGallery ${CinderSamples_LINK_LIBRARIES})