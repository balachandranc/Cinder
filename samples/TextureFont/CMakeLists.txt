include_directories(include)
add_executable(TextureFont
	src/TextureFontApp.cpp
)
target_link_libraries(TextureFont ${CinderSamples_LINK_LIBRARIES})