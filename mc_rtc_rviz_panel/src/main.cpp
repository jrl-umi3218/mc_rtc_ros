#include "Panel.h"

#include <QApplication>
#include <QMainWindow>
#include <QVBoxLayout>

#include <QPushButton>

#include <csignal>

QApplication * app_ = nullptr;

void signal_handler(int sig)
{
  if(app_) { app_->exit(); }
}

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "mc_rtc_gui");
  QApplication app(argc, argv);
  app_ = &app;
  QMainWindow window;
  window.setWindowTitle("mc_rtc GUI");
  window.setCentralWidget(new mc_rtc_rviz::Panel(&window));
  window.show();
  std::signal(SIGINT, &signal_handler);
  return app.exec();
}
