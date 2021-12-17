cmake_minimum_required(VERSION 3.5.0)
project(gcode_rviz)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rviz
	pluginlib
)

# Qt Stuff
if(rviz_QT_VERSION VERSION_LESS "5")
	find_package(Qt4 ${rviz_QT_VERSION} REQUIRED QtCore QtGui)
	include(${QT_USE_FILE})
	macro(qt_wrap_ui)
		qt4_wrap_ui(${ARGN})
	endmacro()
	macro(qt_wrap_cpp)
		qt4_wrap_cpp(${ARGN})
	endmacro()
else()
	find_package(Qt5 ${rviz_QT_VERSION} REQUIRED Core Widgets)
	set(QT_LIBRARIES Qt5::Widgets)
	macro(qt_wrap_ui)
		qt5_wrap_ui(${ARGN})
	endmacro()
	macro(qt_wrap_cpp)
		qt5_wrap_cpp(${ARGN})
	endmacro()
endif()

add_definitions(-DQT_NO_KEYWORDS)

catkin_package(
	INCLUDE_DIRS
		include
		${QT_INCLUDE_DIR}
	LIBRARIES
		${PROJECT_NAME}_render_tools
	CATKIN_DEPENDS
		roscpp
		rviz
		pluginlib
	DEPENDS
		QT
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	SYSTEM ${QT_INCLUDE_DIR}
)

# Rviz Render Tools
qt_wrap_cpp(${PROJECT_NAME}_render_tools_cpp_MOCS
	include/gcode_rviz/render_tools/open_gcode_panel.h)

add_library(${PROJECT_NAME}_render_tools SHARED
	src/render_tools/open_gcode_panel.cpp
	${${PROJECT_NAME}_render_tools_cpp_MOCS})

target_link_libraries(${PROJECT_NAME}_render_tools PUBLIC
	${catkin_LIBRARIES}
	${QT_LIBRARIES})

target_include_directories(${PROJECT_NAME}_render_tools PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_render_tools SYSTEM PUBLIC
	${catkin_INCLUDE_DIRS}
	${QT_INCLUDE_DIR})

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_render_tools
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# Mark plugin XML files for installation
install(FILES gcode_rviz_test_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
