/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include "Qt.h"

#include <mc_control/ControllerClient.h>

#ifdef MC_RTC_ROS_IS_ROS2
#  include <rclcpp/time.hpp>
#  include <rviz_common/display_context.hpp>
#  include <rviz_common/display_group.hpp>
#else
#  include <ros/time.h>
#  include <rviz/display_context.h>
#  include <rviz/display_group.h>
#endif

namespace mc_rtc_rviz
{

using WidgetId = mc_control::ElementId;

#ifdef MC_RTC_ROS_IS_ROS2
using Time = rclcpp::Time;
using DisplayGroup = rviz_common::DisplayGroup;
using DisplayContext = rviz_common::DisplayContext;
#else
using Time = ros::Time;
using DisplayGroup = rviz::DisplayGroup;
using DisplayContext = rviz::DisplayContext;
#endif

struct ClientWidgetParam
{
  mc_control::ControllerClient & client;
  QWidget * parent;
  const WidgetId & id;
};

std::string id2name(const WidgetId & id);

/** Base class for all widget handled by the controller client */
struct ClientWidget : public QWidget
{
public:
  ClientWidget(const ClientWidgetParam & params);

  virtual ~ClientWidget() = default;

  /** Name of the widget */
  const std::string & name() const { return id_.name; }

  /** If a widget name starts with # then it should be displayed discreetly */
  bool secret() const { return name().size() == 0 || name()[0] == '#'; }

  /** Id of the widget */
  const WidgetId & id() const { return id_; }

  /** Stack id of the widget */
  int sid() const { return id().sid; }

  /** Return true if the element was seen this round */
  virtual bool wasSeen() { return seen_; }

  /** Called to set seen status back to false */
  virtual void resetSeen() { seen_ = false; }

  /** Called to set seen status to true */
  void seen() { seen_ = true; }

  /** Get the element visibility stored in the configuration */
  bool visible();

  /** Set the element visibility stored in the configuration */
  void visible(bool visibility);

  /** Add a show/hide button to the widget for 3D elements
   *
   * To be implemented when relevant, the default implementation does nothing
   */
  virtual QPushButton * showHideButton() { return nullptr; }

  /** To be implemented for containers, default implementation throws */
  virtual void addWidget(ClientWidget * w);

  /** To be implemented for containers, default implementation throws */
  virtual void removeWidget(ClientWidget * w);

  /** To be implemented in containers, returns the number of remaining elements */
  virtual size_t clean() { return 1; }

  /** To be implemented in container, return a widget by its name or a nullptr if the widget does not exist */
  virtual ClientWidget * widget(const std::string &) { return nullptr; }

  /** Get the current ROS time */
  Time now() const;

  inline void setDisplayContext(DisplayContext * display_context) { display_context_ = display_context; }
  inline void setDisplayGroup(DisplayGroup * display_group) { root_display_group_ = display_group; };

protected:
  mc_control::ControllerClient & client() { return client_; }
  DisplayGroup * displayGroup() { return root_display_group_; }
  DisplayContext * displayContext() { return display_context_; }

private:
  mc_control::ControllerClient & client_;
  DisplayGroup * root_display_group_ = nullptr;
  DisplayContext * display_context_ = nullptr;
  WidgetId id_;
  bool seen_ = true;
};

} // namespace mc_rtc_rviz
