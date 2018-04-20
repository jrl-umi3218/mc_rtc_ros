set(MOC_HEADERS
  Panel.h
  ArrayInputWidget.h
  ButtonWidget.h
  CategoryWidget.h
  CheckboxWidget.h
  ComboInputWidget.h
  FormElement.h
  FormWidget.h
  GenericInputWidget.h
  InteractiveMarkerWidget.h
  SchemaWidget.h
)

# Stricty speaking we'd want to check <= indigo but the goal is to support 14.04 and strictly go forward from that
if(NOT DEFINED ENV{MC_RTC_RVIZ_PANEL_FORCE_QT5})
  set(FORCE_QT5 0)
else()
  set(FORCE_QT5 $ENV{MC_RTC_RVIZ_PANEL_FORCE_QT5})
endif()
if(NOT ${FORCE_QT5} AND $ENV{ROS_DISTRO} STREQUAL "indigo")
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  qt4_wrap_cpp(MOC_FILES
    ${MOC_HEADERS}
  )
else()
  find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)
  qt5_wrap_cpp(MOC_FILES
    ${MOC_HEADERS}
  )
endif()

set(HEADER_FILES
  ArrayLabelWidget.h
  ClientWidget.h
  LabelWidget.h
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
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
pkg_config_use_dependency(${PROJECT_NAME} mc_rtc)
target_link_libraries(${PROJECT_NAME} mc_control_client ${catkin_LIBRARIES})
target_compile_definitions(${PROJECT_NAME} PRIVATE -DMC_RTC_DOCDIR="${MC_RTC_DOCDIR}")
if(NOT ${FORCE_QT5} AND $ENV{ROS_DISTRO} STREQUAL "indigo")
  target_link_libraries(${PROJECT_NAME} Qt4::QtCore Qt4::QtGui)
else()
  target_link_libraries(${PROJECT_NAME} Qt5::Core Qt5::Gui Qt5::Widgets)
endif()

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
