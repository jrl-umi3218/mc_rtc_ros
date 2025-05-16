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
    SchemaWidget.h
    TableWidget.h
    InteractiveMarkerWidget.h
    ArrowInteractiveMarkerWidget.h
    ForceInteractiveMarkerWidget.h
    Point3DInteractiveMarkerWidget.h
    TransformInteractiveMarkerWidget.h
    VisualWidget.h
    XYThetaInteractiveMarkerWidget.h
    DisplayTrajectoryWidget.h
    PolygonMarkerWidget.h
    PolyhedronMarkerWidget.h
    plugin.h
    RobotModelDisplay.h)

if(NOT DEFINED ENV{ROS_DISTRO})
  set(ROS_DISTRO "none")
else()
  set(ROS_DISTRO "$ENV{ROS_DISTRO}")
endif()

# Stricty speaking we'd want to check <= indigo but the goal is to support 14.04
# and strictly go forward from that
if(NOT DEFINED ENV{MC_RTC_RVIZ_PANEL_FORCE_QT5})
  set(FORCE_QT5 0)
else()
  set(FORCE_QT5 $ENV{MC_RTC_RVIZ_PANEL_FORCE_QT5})
endif()
if(NOT ${FORCE_QT5} AND "${ROS_DISTRO}" STREQUAL "indigo")
  find_package(
    Qt4
    COMPONENTS QtCore QtGui
    REQUIRED)
  qt4_wrap_cpp(MOC_FILES ${MOC_HEADERS})
  qt4_add_resources(QRC_FILES icons.qrc)
else()
  find_package(
    Qt5
    COMPONENTS Core Gui Widgets
    REQUIRED)
  qt5_wrap_cpp(MOC_FILES ${MOC_HEADERS})
  qt5_add_resources(QRC_FILES icons.qrc)
endif()

set(HEADER_FILES
    ArrayLabelWidget.h
    ClientWidget.h
    ConnectionConfiguration.h
    FormElementContainer.h
    LabelWidget.h
    Schema.h
    utils.h)

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
    FormElementContainer.cpp
    FormWidget.cpp
    GenericInputWidget.cpp
    LabelWidget.cpp
    NumberSliderWidget.cpp
    PlotWidget.cpp
    PlotTabWidget.cpp
    Schema.cpp
    SchemaArrayInput.cpp
    SchemaWidget.cpp
    TableWidget.cpp
    InteractiveMarkerWidget.cpp
    ArrowInteractiveMarkerWidget.cpp
    ForceInteractiveMarkerWidget.cpp
    Point3DInteractiveMarkerWidget.cpp
    TransformInteractiveMarkerWidget.cpp
    VisualWidget.cpp
    XYThetaInteractiveMarkerWidget.cpp
    DisplayTrajectoryWidget.cpp
    PolygonMarkerWidget.cpp
    PolyhedronMarkerWidget.cpp
    utils.cpp
    plugin.cpp
    RobotModelDisplay.cpp
    ${MOC_FILES}
    ${QRC_FILES})

if(NOT DEFINED MC_RTC_DOCDIR)
  set(MC_RTC_DOCDIR "${mc_rtc_DIR}/../../../share/doc/mc_rtc")
endif()
get_filename_component(MC_RTC_DOCDIR "${MC_RTC_DOCDIR}" ABSOLUTE)
if(NOT EXISTS "${MC_RTC_DOCDIR}")
  message(
    SEND_ERROR
      "Provided mc_rtc documentation directory (${MC_RTC_DOCDIR}) does not exist"
  )
endif()

add_library(_gui_objects OBJECT ${SOURCE_FILES})
target_compile_definitions(_gui_objects
                           PRIVATE -DMC_RTC_DOCDIR="${MC_RTC_DOCDIR}")
if(NOT ${FORCE_QT5} AND "${ROS_DISTRO}" STREQUAL "indigo")
  target_link_libraries(_gui_objects PUBLIC Qt4::QtCore Qt4::QtGui)
else()
  target_link_libraries(_gui_objects PUBLIC Qt5::Core Qt5::Gui Qt5::Widgets)
endif()
target_compile_options(_gui_objects PRIVATE -fPIC)
target_link_libraries(_gui_objects PUBLIC Qwt::Qwt mc_rtc::mc_control_client
                                          mc_rtc::mc_rtc_ros)
if(ROS_VERSION EQUAL 1)
  add_dependencies(_gui_objects ${catkin_EXPORTED_TARGETS})
  target_link_libraries(_gui_objects PUBLIC ${catkin_LIBRARIES})
elseif(ROS_VERSION EQUAL 2)
  target_link_libraries(_gui_objects PUBLIC ROSDependencies)
endif()

add_library(${PROJECT_NAME} SHARED Panel.cpp ${HEADER_FILES} ${MOC_HEADERS})
if(ROS_VERSION EQUAL 2)
  target_compile_definitions(
    ${PROJECT_NAME} PRIVATE RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY
                            MC_RTC_RVIZ_PANEL_IS_ROS2)
endif()
target_link_libraries(${PROJECT_NAME} PUBLIC _gui_objects)

add_executable(mc_rtc_gui main.cpp)
target_link_libraries(mc_rtc_gui PUBLIC ${PROJECT_NAME})
target_compile_options(mc_rtc_gui PRIVATE -fPIC)

if(ROS_VERSION EQUAL 1)
  install(
    TARGETS ${PROJECT_NAME} mc_rtc_gui
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
elseif(ROS_VERSION EQUAL 2)
  install(
    TARGETS ${PROJECT_NAME} mc_rtc_gui
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
endif()
