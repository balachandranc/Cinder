include_directories(include)
add_executable(Wisteria
	src/Branch.cpp
	src/WisteriaApp.cpp
)
target_link_libraries(Wisteria ${CinderSamples_LINK_LIBRARIES})