find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)

include_directories(/usr/include/x86_64-linux-gnu/qt5)
include_directories(/usr/include/x86_64-linux-gnu/qt5/QtCore)
include_directories(/usr/include/x86_64-linux-gnu/qt5/QtWidgets)
include_directories(/usr/include/x86_64-linux-gnu/qt5/QtGui)

qt5_wrap_cpp(MOC_FILES
  Panel.h
)

set(HEADER_FILES
  BaseWidget.h
  ButtonWidget.h
  FormWidget.h
  InputWidget.h
  LabelWidget.h
  Point3DWidget.h
  PointInputDialog.h
  SchemaWidget.h
  SchemaForm.h
  TransformWidget.h
  utils.h
)

set(SOURCE_FILES
  Panel.cpp
  ButtonWidget.cpp
  FormWidget.cpp
  InputWidget.cpp
  LabelWidget.cpp
  PointInputDialog.cpp
  Point3DWidget.cpp
  SchemaWidget.cpp
  SchemaForm.cpp
  TransformWidget.cpp
  utils.cpp
  ${MOC_FILES}
)

add_library(${PROJECT_NAME} ${SOURCE_FILES} ${HEADER_FILES})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
pkg_config_use_dependency(${PROJECT_NAME} mc_rtc)
target_link_libraries(${PROJECT_NAME} mc_control_client)
target_compile_definitions(${PROJECT_NAME} PRIVATE -DMC_RTC_DOCDIR="${MC_RTC_DOCDIR}")

add_executable(mc_rtc_gui main.cpp)
target_link_libraries(mc_rtc_gui ${PROJECT_NAME} ${QT_LIBRARIES} Qt5Widgets Qt5Core)
target_compile_options(mc_rtc_gui PRIVATE -fPIC)

install(
  TARGETS
  ${PROJECT_NAME} mc_rtc_gui
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  FILES
  rviz_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
