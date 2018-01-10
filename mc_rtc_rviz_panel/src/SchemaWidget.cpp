#include "SchemaWidget.h"
#include "SchemaForm.h"

#include <iostream>

std::string schema_base = std::string(MC_RTC_DOCDIR) + "/json/schemas/";

SchemaWidget::SchemaWidget(QWidget * parent, const mc_rtc::Configuration & data,
    request_t request)
: BaseWidget(new QVBoxLayout(), parent),
  request_(request),
  combo_(new QComboBox(this)),
  scroll_(new QScrollArea(this)),
  stack_(new QStackedWidget(this))
{
  stack_->addWidget(new QWidget());
  scroll_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  scroll_->setMaximumHeight(1000);
  scroll_->setWidget(stack_);
  scroll_->setWidgetResizable(true);
  combo_->addItem("Please select a task in the list...");
  void (QComboBox::*sig)(int) = &QComboBox::currentIndexChanged;
  layout->addWidget(combo_);
  layout->addWidget(scroll_);
  /*FIXME Fill combo box */
  std::string schema_dir = schema_base + static_cast<std::string>(data("GUI")("schema_dir"));
  QDirIterator dIt(schema_dir.c_str(), {"*.json"}, QDir::Files | QDir::NoDotAndDotDot);
  std::vector<std::string> files;
  while(dIt.hasNext())
  {
    files.push_back(dIt.next().toStdString());
  }
  std::sort(files.begin(), files.end());
  for(const auto & f : files)
  {
    auto w = new SchemaForm(f);
    stack_->addWidget(w);
    combo_->addItem(w->title.c_str());
  }
  layout->addWidget(confirm_button_);
  scroll_->hide();
  confirm_button_->hide();
  static_cast<QVBoxLayout*>(layout)->addStretch(1);
  setLayout(layout);
  /** Connections */
  connect(combo_, sig,
          this, &SchemaWidget::comboCurrentIndexChanged);
  connect(confirm_button_, &QPushButton::released,
          this, &SchemaWidget::confirmPushed);
}

void SchemaWidget::comboCurrentIndexChanged(int idx)
{
  stack_->setCurrentWidget(stack_->widget(idx));
  stack_->currentWidget()->setMaximumWidth(scroll_->width());
  if(idx != 0)
  {
    scroll_->show();
    stack_->currentWidget()->setMaximumWidth(scroll_->width()  - 20);
    confirm_button_->show();
  }
  else
  {
    scroll_->hide();
    confirm_button_->hide();
  }
}

void SchemaWidget::confirmPushed()
{
  auto idx = combo_->currentIndex();
  if(idx == 0) return;
  auto w = static_cast<SchemaForm*>(stack_->currentWidget());
  auto ready = w->ready();
  if(ready.size() == 0)
  {
    request_(w->send());
  }
  else
  {
    /** FIXME Display error box */
    std::cerr << "Form is not complete" << std::endl << ready << std::endl;
  }
}
