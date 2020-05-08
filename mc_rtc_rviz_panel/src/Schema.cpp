#include "Schema.h"

#include <mc_rtc/logging.h>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

namespace mc_rtc_rviz
{

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
      if(!store().count(ref_schema.string()))
      {
        store()[ref_schema.string()] = Schema{ref_schema.string()};
      }
      auto cf = create_form;
      bool required = is_required(k);
      std::string ref_schema_str = ref_schema.string();
      create_form = [cf, k, required, ref_schema_str](QWidget * parent, const mc_rtc::Configuration & data) {
        auto v = cf(parent, data);
        const auto & schema = store().at(ref_schema_str);
        if(schema.is_object())
        {
          v.emplace_back(new form::Form(parent, k, required, store().at(ref_schema_str).create_form(parent, data)));
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

SchemaStore & Schema::store()
{
  static SchemaStore store;
  return store;
}

SchemaArrayInput::SchemaArrayInput(QWidget * parent,
                                   Schema schema,
                                   bool required,
                                   bool fixed_size,
                                   int min_size,
                                   int max_size)
: FormElement(parent, schema.title(), required)
{
}

void SchemaArrayInput::fill_(mc_rtc::Configuration & out) {}

} // namespace mc_rtc_rviz
