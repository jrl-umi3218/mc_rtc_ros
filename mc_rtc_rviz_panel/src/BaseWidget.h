#pragma once

#include <QtWidgets>

#include <mc_rtc/Configuration.h>

/** A function used to send a request to the server */
using request_t = std::function<void(const mc_rtc::Configuration&)>;

/** The base widget for a given input */
struct BaseWidget : public QWidget
{
  /** Constructor */
  template<typename Layout = QHBoxLayout>
  BaseWidget(Layout * layout = nullptr, QWidget * parent = nullptr) : QWidget(parent), layout(layout ? layout : new Layout()) { setLayout(this->layout); }

  virtual ~BaseWidget() = default;

  /** Called with the configuration object holding information for this widget */
  virtual void update(const mc_rtc::Configuration&) = 0;

  /** Layout of the widget */
  QLayout * layout;
};
