#pragma once

#include <QtGlobal>
#if QT_VERSION < QT_VERSION_CHECK(5, 0, 0)
#include <QtGui>
#else
#include <QtWidgets>
#endif

#include <mc_control/ControllerClient.h>

namespace mc_rtc_rviz
{

using WidgetId = mc_control::ElementId;

struct ClientWidgetParam
{
  mc_control::ControllerClient & client;
  QWidget * parent;
  const WidgetId & id;
};

std::string id2name(const WidgetId& id);

/** Base class for all widget handled by the controller client */
struct ClientWidget : public QWidget
{
public:
  ClientWidget(const ClientWidgetParam & params);

  virtual ~ClientWidget() = default;

  /** Name of the widget */
  const std::string & name() const { return id_.name; }

  /** Id of the widget */
  const WidgetId & id() { return id_; }

  /** Return the value of seen and set it back to false */
  bool seen();

  /** Set seen value */
  void seen(bool s) { seen_ = s; }

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
  virtual ClientWidget * widget(const std::string & name) { return nullptr; }
protected:
  mc_control::ControllerClient & client() { return client_; }
private:
  mc_control::ControllerClient & client_;
  WidgetId id_;
  bool seen_ = true;
};

}
