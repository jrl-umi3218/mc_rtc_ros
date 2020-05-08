/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "SchemaWidget.h"

#include "FormElement.h"
#include "FormWidget.h"

#include <mc_rtc/logging.h>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

namespace mc_rtc_rviz
{

namespace
{

struct Schema;

using SchemaStore = std::map<std::string, Schema>;
using Form = std::vector<FormElement *>;
using FormMaker = std::function<Form(QWidget *, const mc_rtc::Configuration &)>;

struct Schema
{
  Schema() = default;
  Schema(const std::string & file);
  Schema(const mc_rtc::Configuration & data, const std::string & source);

  const std::string & title() const
  {
    return title_;
  }

  bool is_object() const
  {
    return is_object_;
  }

  FormMaker create_form = [](QWidget *, const mc_rtc::Configuration &) -> Form { return {}; };

private:
  std::string title_;
  bool is_object_ = false;
};

static SchemaStore store = {};
static const std::string schema_dir = std::string(MC_RTC_DOCDIR) + "/json/schemas/";

Schema::Schema(const std::string & file) : Schema(mc_rtc::Configuration{file}, file) {}

Schema::Schema(const mc_rtc::Configuration & s, const std::string & source)
{
  title_ = s("title", "No title property in " + source);
  auto required = s("required", std::vector<std::string>{});
  auto is_required = [&required](const std::string & label) {
    return std::find(required.begin(), required.end(), label) != required.end();
  };
  /** Handle enum entries */
  auto handle_enum = [this](const std::string & k, bool required, const std::vector<std::string> & values) {
    auto cf = create_form;
    create_form = [cf, k, required, values](QWidget * parent, const mc_rtc::Configuration & data) {
      auto v = cf(parent, data);
      v.emplace_back(new form::ComboInput(parent, k, required, values, false));
      if(values.size() == 1 && required)
      {
        v.back()->hidden(true);
      }
      return v;
    };
  };
  /** Handle: boolean/integer/number/string */
  auto handle_type = [this, &source](const std::string & k, bool required, const std::string & type,
                                     mc_rtc::Configuration & schema) {
    auto cf = create_form;
    if(type == "boolean")
    {
      create_form = [cf, k, required](QWidget * parent, const mc_rtc::Configuration & data) {
        auto v = cf(parent, data);
        v.emplace_back(new form::Checkbox(parent, k, required, true));
        return v;
      };
    }
    else if(type == "integer")
    {
      if(k == "robotIndex")
      {
        create_form = [cf, required](QWidget * parent, const mc_rtc::Configuration & data) {
          auto v = cf(parent, data);
          v.emplace_back(new form::DataComboInput(parent, "robot", required, data, {"robots"}, true, "robotIndex"));
          return v;
        };
        return;
      }
      create_form = [cf, k, required](QWidget * parent, const mc_rtc::Configuration & data) {
        auto v = cf(parent, data);
        v.emplace_back(new form::IntegerInput(parent, k, required, 0));
        return v;
      };
    }
    else if(type == "number")
    {
      create_form = [cf, k, required](QWidget * parent, const mc_rtc::Configuration & data) {
        auto v = cf(parent, data);
        v.emplace_back(new form::NumberInput(parent, k, required, 0));
        return v;
      };
    }
    else if(type == "object")
    {
      if(!schema.has("title"))
      {
        schema.add("title", k);
      }
      Schema s(schema, source);
      create_form = [cf, k, required, s](QWidget * parent, const mc_rtc::Configuration & data) {
        auto v = cf(parent, data);
        v.emplace_back(new form::Form(parent, k, required, s.create_form(parent, data)));
        return v;
      };
    }
    else
    {
      if(type != "string")
      {
        LOG_WARNING("Property " << k << " in " << source << " has unknown or missing type (value: " << type
                                << "), treating as string")
      }
      if(k == "body")
      {
        create_form = [cf, k, required](QWidget * parent, const mc_rtc::Configuration & data) {
          auto v = cf(parent, data);
          v.emplace_back(new form::DataComboInput(parent, "body", required, data, {"bodies", "$robot"}, false));
          return v;
        };
      }
      else if(k == "surface")
      {
        create_form = [cf, k, required](QWidget * parent, const mc_rtc::Configuration & data) {
          auto v = cf(parent, data);
          v.emplace_back(new form::DataComboInput(parent, "surface", required, data, {"surfaces", "$robot"}, false));
          return v;
        };
      }
      else
      {
        create_form = [cf, k, required](QWidget * parent, const mc_rtc::Configuration & data) {
          auto v = cf(parent, data);
          v.emplace_back(new form::StringInput(parent, k, required, ""));
          return v;
        };
      }
    }
  };
  /** Handle an array */
  auto handle_array = [this, &source](const std::string & k, bool required, const std::string & type, size_t min,
                                      size_t max) {
    auto cf = create_form;
    if(type == "integer")
    {
      create_form = [cf, k, required, min, max](QWidget * parent, const mc_rtc::Configuration & data) {
        auto v = cf(parent, data);
        v.emplace_back(new form::IntegerArrayInput(parent, k, required, min == max, min, max));
        return v;
      };
    }
    else if(type == "number")
    {
      create_form = [cf, k, required, min, max](QWidget * parent, const mc_rtc::Configuration & data) {
        auto v = cf(parent, data);
        v.emplace_back(new form::NumberArrayInput(parent, k, required, min == max, min, max));
        return v;
      };
    }
    else if(type == "array")
    {
      // We do nothing here, arrays of arrays are too difficult to handle in the GUI
    }
    else
    {
      if(type != "string")
      {
        LOG_WARNING("Property " << k << " in " << source << " has unknonw or missing array items' type (value: " << type
                                << "), treating as string")
      }
      create_form = [cf, k, required, min, max](QWidget * parent, const mc_rtc::Configuration & data) {
        auto v = cf(parent, data);
        v.emplace_back(new form::StringArrayInput(parent, k, required, min == max, min, max));
        return v;
      };
    }
  };
  std::string type = s("type");
  if(type == "array")
  {
    handle_array(title_, false, s("items", mc_rtc::Configuration{})("type", std::string{""}), s("minItems", 0),
                 s("maxItems", 256));
    return;
  }
  if(type != "object")
  {
    LOG_ERROR(title_ << " from " << source << " has unexpected type: " << type)
    return;
  }
  is_object_ = true;
  auto properties = s("properties", mc_rtc::Configuration{});
  for(const auto & k : properties.keys())
  {
    auto prop = properties(k);
    if(prop.has("enum"))
    {
      handle_enum(k, is_required(k), prop("enum"));
    }
    else if(prop.has("type"))
    {
      std::string type = prop("type");
      if(type == "array")
      {
        handle_array(k, is_required(k), prop("items", mc_rtc::Configuration{})("type", std::string{""}),
                     prop("minItems", 0), prop("maxItems", 256));
      }
      else
      {
        handle_type(k, is_required(k), type, prop);
      }
    }
    else if(prop.has("$ref"))
    {
      std::string ref = prop("$ref");
      ref = ref.substr(3); // remove leading /..
      bfs::path ref_schema{source};
      ref_schema = bfs::canonical(ref_schema.parent_path() / ref);
      if(!store.count(ref_schema.string()))
      {
        store[ref_schema.string()] = Schema{ref_schema.string()};
      }
      auto cf = create_form;
      bool required = is_required(k);
      std::string ref_schema_str = ref_schema.string();
      create_form = [cf, k, required, ref_schema_str](QWidget * parent, const mc_rtc::Configuration & data) {
        auto v = cf(parent, data);
        const auto & schema = store.at(ref_schema_str);
        if(schema.is_object())
        {
          v.emplace_back(new form::Form(parent, k, required, store.at(ref_schema_str).create_form(parent, data)));
        }
        else
        {
          auto el = schema.create_form(parent, data).at(0);
          el->name(k);
          v.push_back(el);
        }
        return v;
      };
    }
    else
    {
      LOG_ERROR("Cannot handle property " << k << " in " << source << ": " << prop.dump())
    }
  }
}

} // namespace

SchemaWidget::SchemaWidget(const ClientWidgetParam & params,
                           const std::string & schema,
                           const mc_rtc::Configuration & data)
: ClientWidget(params)
{
  bfs::path schema_path = schema_dir;
  schema_path /= schema;
  if(!bfs::exists(schema_path))
  {
    LOG_ERROR("Schema path: " << schema_path.string() << " does not exist in this machine")
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
    store[path.string()] = s;
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
