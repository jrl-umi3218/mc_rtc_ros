/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include "ClientWidget.h"

#include <unordered_map>

namespace mc_rtc_rviz
{

struct PlotWidget;
struct PlotTabWidget;

struct PopTabButton : public QPushButton
{
  Q_OBJECT
public:
  PopTabButton(int index, PlotTabWidget * tab, QWidget * parent);

  void update(int idx);

private:
  int index_;
  PlotTabWidget * tab_;
private slots:
  void clicked();
};

struct CloseTabButton : public QPushButton
{
  Q_OBJECT
public:
  CloseTabButton(int index, PlotTabWidget * tab, QWidget * parent);

  void update(int idx);

private:
  int index_;
  PlotTabWidget * tab_;
private slots:
  void clicked();
};

struct PlotTabBar : public QTabBar
{
  Q_OBJECT
public:
  PlotTabBar(QWidget * parent, PlotTabWidget * tab);

  void mouseReleaseEvent(QMouseEvent * event) override;

  void tabInserted(int index) override;

  void tabLayoutChange() override;

private:
  PlotTabWidget * tab_;
};

struct PlotTab : public QTabWidget
{
  Q_OBJECT
public:
  PlotTab(QWidget * parent);

  void setTabBar(QTabBar * bar);
};

struct PlotDialog : public QDialog
{
  Q_OBJECT
public:
  PlotDialog(PlotWidget * plot, PlotTabWidget * parent, QString title);

  void closeEvent(QCloseEvent * event) override;

  PlotWidget * plot()
  {
    return plot_;
  }

private:
  PlotWidget * plot_;
  PlotTabWidget * parent_;
private slots:
  void handle_close();
};

struct PlotTabWidget : public ClientWidget
{
  Q_OBJECT
public:
  PlotTabWidget(const ClientWidgetParam & param);

  bool wasSeen() override;

  void resetSeen() override;

  void start_plot(uint64_t id, const std::string & title);

  void plot_setup_xaxis(uint64_t id, const std::string & legend, const mc_rtc::gui::plot::Range & range);

  void plot_setup_yaxis_left(uint64_t id, const std::string & legend, const mc_rtc::gui::plot::Range & range);

  void plot_setup_yaxis_right(uint64_t id, const std::string & legend, const mc_rtc::gui::plot::Range & range);

  void plot_point(uint64_t id,
                  uint64_t did,
                  const std::string & legend,
                  double x,
                  double y,
                  mc_rtc::gui::Color color,
                  mc_rtc::gui::plot::Style style,
                  mc_rtc::gui::plot::Side side);

  void plot_polygon(uint64_t id,
                    uint64_t did,
                    const std::string & legend,
                    const mc_rtc::gui::plot::PolygonDescription & polygon,
                    mc_rtc::gui::plot::Side side);

  void plot_polygons(uint64_t id,
                     uint64_t did,
                     const std::string & legend,
                     const std::vector<mc_rtc::gui::plot::PolygonDescription> & polygons,
                     mc_rtc::gui::plot::Side side);

  void end_plot(uint64_t id);

  bool remove_plot_widget(PlotWidget * widget);

  void add_plot_widget(QString title, PlotWidget * widget);

  void close_dialog(PlotDialog * dialog);

  PlotTab * tab()
  {
    return tab_;
  }

private:
  /** Holds plots in tabs */
  PlotTab * tab_;
  /** Plots */
  std::unordered_map<uint64_t, PlotWidget *> plots_;
  /** Inactive plots */
  std::vector<PlotWidget *> inactive_plots_;
  /** Active dialogs */
  std::vector<PlotDialog *> dialogs_;
  /** Seen plots this round */
  std::unordered_map<uint64_t, bool> seen_;
public slots:
  void popTab(int idx);
private slots:
  void closeTabOnRequest(int i);
  void previousTab();
  void nextTab();
  void closeCurrentTab();
};

} // namespace mc_rtc_rviz
