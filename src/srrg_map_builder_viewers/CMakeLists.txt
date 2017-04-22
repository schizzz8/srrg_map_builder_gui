add_library(srrg_map_builder_viewers_library SHARED   
	map_builder_viewer.cpp map_builder_viewer.h  
) 

target_link_libraries(srrg_map_builder_viewers_library   
	${catkin_LIBRARIES}
  	${QGLVIEWER_LIBRARY} 
  	${SRRG_QT_LIBRARIES} 
  	${OPENGL_gl_LIBRARY} 
  	${OPENGL_glu_LIBRARY}   
) 
