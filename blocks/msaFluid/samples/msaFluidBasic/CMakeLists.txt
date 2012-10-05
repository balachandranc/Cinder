include_directories(include)
add_executable( msaFluidBasic
	src/msaFluidBasicApp.cpp
)
target_link_libraries(msaFluidBasic 
	${CINDER_BLOCKS_LIB_DIR}/msaFluid/libmsafluid.a
	${CinderBlocks_LINK_LIBRARIES}
	)