find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)

qt5_wrap_cpp(MOC_FILES
  Panel.h
)

set(HEADER_FILES
  ArrayLabelWidget.h
  ArrayInputWidget.h
  ButtonWidget.h
  CategoryWidget.h
  CheckboxWidget.h
  ClientWidget.h
  ComboInputWidget.h
  FormElement.h
  FormWidget.h
  GenericInputWidget.h
  InteractiveMarkerWidget.h
  LabelWidget.h
  SchemaWidget.h
  utils.h
)

set(SOURCE_FILES
  ArrayLabelWidget.cpp
  ArrayInputWidget.cpp
  ButtonWidget.cpp
  CategoryWidget.cpp
  CheckboxWidget.cpp
  ClientWidget.cpp
  ComboInputWidget.cpp
  FormElement.cpp
  FormWidget.cpp
  GenericInputWidget.cpp
  InteractiveMarkerWidget.cpp
  LabelWidget.cpp
  Panel.cpp
  SchemaWidget.cpp
  utils.cpp
  ${MOC_FILES}
)

add_library(${PROJECT_NAME} ${SOURCE_FILES} ${HEADER_FILES})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
pkg_config_use_dependency(${PROJECT_NAME} mc_rtc)
target_link_libraries(${PROJECT_NAME} mc_control_client)
target_compile_definitions(${PROJECT_NAME} PRIVATE -DMC_RTC_DOCDIR="${MC_RTC_DOCDIR}")
qt5_use_modules(${PROJECT_NAME} Core Gui Widgets)

add_executable(mc_rtc_gui main.cpp)
target_link_libraries(mc_rtc_gui ${PROJECT_NAME})
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
