include_directories(include)
add_executable(FBOBasic src/FBOBasicApp.cpp)
target_link_libraries(FBOBasic ${CinderSamples_LINK_LIBRARIES})