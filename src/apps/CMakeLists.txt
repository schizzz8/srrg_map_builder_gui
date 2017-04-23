add_executable(srrg_map_builder_gui_app srrg_map_builder_gui_app.cpp)
target_link_libraries(srrg_map_builder_gui_app 
  srrg_map_builder_viewers_library
  ${catkin_LIBRARIES}
#  ${QGLVIEWER_LIBRARY}
#  ${SRRG_QT_LIBRARIES}
#  ${OPENGL_gl_LIBRARY}
#  ${OPENGL_glu_LIBRARY}
)
