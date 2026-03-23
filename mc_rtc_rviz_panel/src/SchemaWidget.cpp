/*
 * Copyright 2016-2026 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "SchemaWidget.h"

#include "FormWidget.h"
#include "Schema.h"

#include <mc_rtc/config.h>
#include <mc_rtc/logging.h>

#include <filesystem>
namespace fs = std::filesystem;

namespace mc_rtc_rviz
{

SchemaWidget::SchemaWidget(const ClientWidgetParam & params,
                           const std::string & schema,
                           const mc_rtc::Configuration & dataIn)
: ClientWidget(params)
{
  auto schema_path = fs::path(mc_rtc::MC_RTC_JSON_SCHEMA_PATH);
  schema_path /= schema;
  if(!fs::exists(schema_path))
  {
    mc_rtc::log::error("Schema path: {} does not exist in this machine", schema_path.string());
    return;
  }
  fs::directory_iterator dit(schema_path), endit;
  std::vector<fs::path> drange;
  std::copy(dit, endit, std::back_inserter(drange));
  std::sort(std::begin(drange), std::end(drange));

  auto layout = new QVBoxLayout(this);
  auto combo = new QComboBox(this);
  stack_ = new QStackedWidget(this);
  stack_->addWidget(new QWidget(this));
  for(const auto & p : drange)
  {
    auto path = fs::canonical(p);
    Schema s{path.string()};
    Schema::store()[path.string()] = s;
    auto form = new FormWidget(params);
    for(auto el : s.create_form(this, dataIn)) { form->container()->add_element(el); }
    form->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    combo->addItem(s.title().c_str());
    stack_->addWidget(form);
  }
  combo->setCurrentIndex(-1);
  stack_->setCurrentIndex(-1);
  connect(combo, SIGNAL(currentIndexChanged(int)), this, SLOT(currentIndexChanged(int)));
  layout->addWidget(combo);
  layout->addWidget(stack_);
}

void SchemaWidget::currentIndexChanged(int idx)
{
  stack_->currentWidget()->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  stack_->setCurrentIndex(idx + 1);
  stack_->currentWidget()->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  stack_->currentWidget()->adjustSize();
  stack_->adjustSize();
}

} // namespace mc_rtc_rviz
