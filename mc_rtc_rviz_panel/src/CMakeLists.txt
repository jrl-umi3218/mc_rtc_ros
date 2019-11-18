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
  SchemaWidget.h
)

if(NOT ${DISABLE_ROS})
  list(APPEND MOC_HEADERS
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
if(NOT ${FORCE_QT5} AND ${ROS_DISTRO} STREQUAL "indigo")
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

if(NOT ${DISABLE_ROS})
  list(APPEND HEADER_FILES utils.h)
endif()

set(SOURCE_FILES
  ArrayLabelWidget.cpp
  ArrayInputWidget.cpp
  ButtonWidget.cpp
  CategoryWidget.cpp
  CheckboxWidget.cpp
  ClientWidget.cpp
  ComboInputWidget.cpp
  ConnectionDialog.cpp
  FormElement.cpp
  FormWidget.cpp
  GenericInputWidget.cpp
  LabelWidget.cpp
  NumberSliderWidget.cpp
  Panel.cpp
  PlotWidget.cpp
  SchemaWidget.cpp
  ${MOC_FILES}
)

if(NOT ${DISABLE_ROS})
  list(APPEND SOURCE_FILES
    InteractiveMarkerWidget.cpp
    ArrowInteractiveMarkerWidget.cpp
    ForceInteractiveMarkerWidget.cpp
    Point3DInteractiveMarkerWidget.cpp
    TransformInteractiveMarkerWidget.cpp
    XYThetaInteractiveMarkerWidget.cpp
    DisplayTrajectoryWidget.cpp
    PolygonMarkerWidget.cpp
    utils.cpp
    plugin.cpp)
endif()

find_package(mc_rtc REQUIRED)

add_library(${PROJECT_NAME} ${SOURCE_FILES} ${HEADER_FILES} ${MOC_HEADERS})
target_link_libraries(${PROJECT_NAME} PUBLIC mc_rtc::mc_control_client)
if(NOT ${DISABLE_ROS})
  add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES})
endif()
set(MC_RTC_DOCDIR "${mc_rtc_DIR}/../../../share/doc/mc_rtc")
get_filename_component(MC_RTC_DOCDIR "${MC_RTC_DOCDIR}" ABSOLUTE)
if(NOT EXISTS "${MC_RTC_DOCDIR}")
  message(SEND_ERROR "Provided mc_rtc documentation directory (${MC_RTC_DOCDIR}) does not exist")
endif()
target_compile_definitions(${PROJECT_NAME} PRIVATE -DMC_RTC_DOCDIR="${MC_RTC_DOCDIR}")
if(NOT ${FORCE_QT5} AND ${ROS_DISTRO} STREQUAL "indigo")
  target_link_libraries(${PROJECT_NAME} PUBLIC Qt4::QtCore Qt4::QtGui qwt)
else()
  target_link_libraries(${PROJECT_NAME} PUBLIC Qt5::Core Qt5::Gui Qt5::Widgets qwt-qt5)
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

if(NOT ${DISABLE_ROS})
  install(
    FILES
    rviz_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
endif()
