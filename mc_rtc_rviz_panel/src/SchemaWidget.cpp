/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "SchemaWidget.h"

#include "FormWidget.h"
#include "Schema.h"

#include <mc_rtc/logging.h>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

namespace mc_rtc_rviz
{

static const std::string schema_dir = std::string(MC_RTC_DOCDIR) + "/json/schemas/";

SchemaWidget::SchemaWidget(const ClientWidgetParam & params,
                           const std::string & schema,
                           const mc_rtc::Configuration & data)
: ClientWidget(params)
{
  bfs::path schema_path = schema_dir;
  schema_path /= schema;
  if(!bfs::exists(schema_path))
  {
    mc_rtc::log::error("Schema path: {} does not exist in this machine", schema_path.string());
    return;
  }
  bfs::directory_iterator dit(schema_path), endit;
  std::vector<bfs::path> drange;
  std::copy(dit, endit, std::back_inserter(drange));
  std::sort(std::begin(drange), std::end(drange));

  auto layout = new QVBoxLayout(this);
  auto combo = new QComboBox(this);
  stack_ = new QStackedWidget(this);
  stack_->addWidget(new QWidget(this));
  for(const auto & p : drange)
  {
    auto path = bfs::canonical(p);
    Schema s{path.string()};
    Schema::store()[path.string()] = s;
    auto form = new FormWidget(params);
    for(auto el : s.create_form(this, data))
    {
      form->add_element(el);
    }
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
