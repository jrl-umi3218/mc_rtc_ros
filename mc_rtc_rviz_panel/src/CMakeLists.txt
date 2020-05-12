#
# Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

set(MOC_HEADERS
  Panel.h
  ArrayInputWidget.h
  ButtonWidget.h
  CategoryWidget.h
  CheckboxWidget.h
  ComboInputWidget.h
  ConnectionDialog.h
  FormElement.h
  FormWidget.h
  GenericInputWidget.h
  NumberSliderWidget.h
  PlotWidget.h
  PlotTabWidget.h
  SchemaArrayInput.h
  SchemaWidget.h)

if(NOT ${DISABLE_ROS})
  set(ROS_MOC_HEADERS
    InteractiveMarkerWidget.h
    ArrowInteractiveMarkerWidget.h
    ForceInteractiveMarkerWidget.h
    Point3DInteractiveMarkerWidget.h
    TransformInteractiveMarkerWidget.h
    XYThetaInteractiveMarkerWidget.h
    DisplayTrajectoryWidget.h
    PolygonMarkerWidget.h
    plugin.h)
endif()

if(NOT DEFINED ENV{ROS_DISTRO})
  set(ROS_DISTRO "none")
else()
  set(ROS_DISTRO "$ENV{ROS_DISTRO}")
endif()

# Stricty speaking we'd want to check <= indigo but the goal is to support 14.04 and strictly go forward from that
if(${DISABLE_ROS})
  set(FORCE_QT5 1)
elseif(NOT DEFINED ENV{MC_RTC_RVIZ_PANEL_FORCE_QT5})
  set(FORCE_QT5 0)
else()
  set(FORCE_QT5 $ENV{MC_RTC_RVIZ_PANEL_FORCE_QT5})
endif()
if(NOT ${FORCE_QT5} AND "${ROS_DISTRO}" STREQUAL "indigo")
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  qt4_wrap_cpp(MOC_FILES ${MOC_HEADERS})
  if(NOT ${DISABLE_ROS})
    qt4_wrap_cpp(ROS_MOC_FILES ${ROS_MOC_HEADERS})
  endif()
  qt4_add_resources(QRC_FILES icons.qrc)
else()
  find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)
  qt5_wrap_cpp(MOC_FILES ${MOC_HEADERS})
  if(NOT ${DISABLE_ROS})
    qt5_wrap_cpp(ROS_MOC_FILES ${ROS_MOC_HEADERS})
  endif()
  qt5_add_resources(QRC_FILES icons.qrc)
endif()

set(HEADER_FILES
  ArrayLabelWidget.h
  ClientWidget.h
  ConnectionConfiguration.h
  LabelWidget.h
  Schema.h
  utils.h)

if(NOT ${DISABLE_ROS})
  list(APPEND HEADER_FILES
    utils.h)
endif()

set(SOURCE_FILES
  ArrayLabelWidget.cpp
  ArrayInputWidget.cpp
  ButtonWidget.cpp
  CategoryWidget.cpp
  CheckboxWidget.cpp
  ClientWidget.cpp
  ComboInputWidget.cpp
  ConnectionConfiguration.cpp
  ConnectionDialog.cpp
  FormElement.cpp
  FormWidget.cpp
  GenericInputWidget.cpp
  LabelWidget.cpp
  NumberSliderWidget.cpp
  PlotWidget.cpp
  PlotTabWidget.cpp
  Schema.cpp
  SchemaArrayInput.cpp
  SchemaWidget.cpp
  ${MOC_FILES}
  ${QRC_FILES})

set(MC_RTC_DOCDIR "${mc_rtc_DIR}/../../../share/doc/mc_rtc")
get_filename_component(MC_RTC_DOCDIR "${MC_RTC_DOCDIR}" ABSOLUTE)
if(NOT EXISTS "${MC_RTC_DOCDIR}")
  message(SEND_ERROR "Provided mc_rtc documentation directory (${MC_RTC_DOCDIR}) does not exist")
endif()

add_library(_gui_objects OBJECT ${SOURCE_FILES})
target_compile_definitions(_gui_objects PRIVATE -DMC_RTC_DOCDIR="${MC_RTC_DOCDIR}")
if(NOT ${FORCE_QT5} AND "${ROS_DISTRO}" STREQUAL "indigo")
  target_include_directories(_gui_objects PRIVATE $<TARGET_PROPERTY:Qt4::QtCore,INTERFACE_INCLUDE_DIRECTORIES>)
  target_compile_definitions(_gui_objects PRIVATE $<TARGET_PROPERTY:Qt4::QtCore,INTERFACE_COMPILE_DEFINITIONS>)
  target_include_directories(_gui_objects PRIVATE $<TARGET_PROPERTY:Qt4::QtGui,INTERFACE_INCLUDE_DIRECTORIES>)
  target_compile_definitions(_gui_objects PRIVATE $<TARGET_PROPERTY:Qt4::QtGui,INTERFACE_COMPILE_DEFINITIONS>)
else()
  target_include_directories(_gui_objects PRIVATE $<TARGET_PROPERTY:Qt5::Core,INTERFACE_INCLUDE_DIRECTORIES>)
  target_compile_definitions(_gui_objects PRIVATE $<TARGET_PROPERTY:Qt5::Core,INTERFACE_COMPILE_DEFINITIONS>)
  target_include_directories(_gui_objects PRIVATE $<TARGET_PROPERTY:Qt5::Gui,INTERFACE_INCLUDE_DIRECTORIES>)
  target_compile_definitions(_gui_objects PRIVATE $<TARGET_PROPERTY:Qt5::Gui,INTERFACE_COMPILE_DEFINITIONS>)
  target_include_directories(_gui_objects PRIVATE $<TARGET_PROPERTY:Qt5::Widgets,INTERFACE_INCLUDE_DIRECTORIES>)
  target_compile_definitions(_gui_objects PRIVATE $<TARGET_PROPERTY:Qt5::Widgets,INTERFACE_COMPILE_DEFINITIONS>)
endif()
target_compile_options(_gui_objects PRIVATE -fPIC)
target_include_directories(_gui_objects PUBLIC $<TARGET_PROPERTY:Qwt::Qwt,INTERFACE_INCLUDE_DIRECTORIES>)
target_include_directories(_gui_objects PUBLIC $<TARGET_PROPERTY:mc_rtc::mc_control_client,INTERFACE_INCLUDE_DIRECTORIES>)
target_compile_definitions(_gui_objects PUBLIC $<TARGET_PROPERTY:mc_rtc::mc_control_client,INTERFACE_COMPILE_DEFINITIONS>)
if(${DISABLE_ROS})
  target_compile_definitions(_gui_objects PUBLIC -DDISABLE_ROS)
endif()

if(NOT ${DISABLE_ROS})
  set(ROS_SOURCE_FILES
    InteractiveMarkerWidget.cpp
    ArrowInteractiveMarkerWidget.cpp
    ForceInteractiveMarkerWidget.cpp
    Point3DInteractiveMarkerWidget.cpp
    TransformInteractiveMarkerWidget.cpp
    XYThetaInteractiveMarkerWidget.cpp
    DisplayTrajectoryWidget.cpp
    PolygonMarkerWidget.cpp
    utils.cpp
    plugin.cpp
    ${ROS_MOC_FILES})
  add_library(_gui_ros_objects OBJECT ${ROS_SOURCE_FILES})
  if(NOT ${FORCE_QT5} AND "${ROS_DISTRO}" STREQUAL "indigo")
    target_include_directories(_gui_ros_objects PRIVATE $<TARGET_PROPERTY:Qt4::QtCore,INTERFACE_INCLUDE_DIRECTORIES>)
    target_compile_definitions(_gui_ros_objects PRIVATE $<TARGET_PROPERTY:Qt4::QtCore,INTERFACE_COMPILE_DEFINITIONS>)
    target_include_directories(_gui_ros_objects PRIVATE $<TARGET_PROPERTY:Qt4::QtGui,INTERFACE_INCLUDE_DIRECTORIES>)
    target_compile_definitions(_gui_ros_objects PRIVATE $<TARGET_PROPERTY:Qt4::QtGui,INTERFACE_COMPILE_DEFINITIONS>)
  else()
    target_include_directories(_gui_ros_objects PRIVATE $<TARGET_PROPERTY:Qt5::Core,INTERFACE_INCLUDE_DIRECTORIES>)
    target_compile_definitions(_gui_ros_objects PRIVATE $<TARGET_PROPERTY:Qt5::Core,INTERFACE_COMPILE_DEFINITIONS>)
    target_include_directories(_gui_ros_objects PRIVATE $<TARGET_PROPERTY:Qt5::Gui,INTERFACE_INCLUDE_DIRECTORIES>)
    target_compile_definitions(_gui_ros_objects PRIVATE $<TARGET_PROPERTY:Qt5::Gui,INTERFACE_COMPILE_DEFINITIONS>)
    target_include_directories(_gui_ros_objects PRIVATE $<TARGET_PROPERTY:Qt5::Widgets,INTERFACE_INCLUDE_DIRECTORIES>)
    target_compile_definitions(_gui_ros_objects PRIVATE $<TARGET_PROPERTY:Qt5::Widgets,INTERFACE_COMPILE_DEFINITIONS>)
  endif()
  target_compile_options(_gui_ros_objects PRIVATE -fPIC)
  target_include_directories(_gui_ros_objects PUBLIC $<TARGET_PROPERTY:mc_rtc::mc_rtc_ros,INTERFACE_INCLUDE_DIRECTORIES>)
  target_compile_definitions(_gui_ros_objects PUBLIC $<TARGET_PROPERTY:mc_rtc::mc_rtc_ros,INTERFACE_COMPILE_DEFINITIONS>)
endif()

if(NOT ${DISABLE_ROS})
  add_library(${PROJECT_NAME} Panel.cpp ${HEADER_FILES} ${MOC_HEADERS} ${ROS_MOC_HEADERS} $<TARGET_OBJECTS:_gui_objects> $<TARGET_OBJECTS:_gui_ros_objects>)
  add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES} mc_rtc::mc_rtc_ros)
  if(${BUILD_ROS_FREE_GUI})
    add_executable(mc_rtc_gui_noros main.cpp Panel.cpp ${HEADER_FILES} ${MOC_HEADERS} $<TARGET_OBJECTS:_gui_objects>)
    target_compile_options(mc_rtc_gui_noros PUBLIC -DDISABLE_ROS)
    if(NOT ${FORCE_QT5} AND "${ROS_DISTRO}" STREQUAL "indigo")
      target_link_libraries(mc_rtc_gui_noros PUBLIC Qt4::QtCore Qt4::QtGui)
    else()
      target_link_libraries(mc_rtc_gui_noros PUBLIC Qt5::Core Qt5::Gui Qt5::Widgets)
    endif()
    target_link_libraries(mc_rtc_gui_noros PUBLIC mc_rtc::mc_control_client Qwt::Qwt)
  endif()
else()
  add_library(${PROJECT_NAME} Panel.cpp ${HEADER_FILES} ${MOC_HEADERS} $<TARGET_OBJECTS:_gui_objects>)
  target_compile_options(${PROJECT_NAME} PUBLIC -DDISABLE_ROS)
endif()

target_link_libraries(${PROJECT_NAME} PUBLIC mc_rtc::mc_control_client Qwt::Qwt)
if(NOT ${FORCE_QT5} AND "${ROS_DISTRO}" STREQUAL "indigo")
  target_link_libraries(${PROJECT_NAME} PUBLIC Qt4::QtCore Qt4::QtGui)
else()
  target_link_libraries(${PROJECT_NAME} PUBLIC Qt5::Core Qt5::Gui Qt5::Widgets)
endif()

add_executable(mc_rtc_gui main.cpp)
target_link_libraries(mc_rtc_gui PUBLIC ${PROJECT_NAME})
target_compile_options(mc_rtc_gui PRIVATE -fPIC)

install(
  TARGETS
  ${PROJECT_NAME} mc_rtc_gui
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(TARGET mc_rtc_gui_noros)
  install(TARGETS mc_rtc_gui_noros DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endif()
