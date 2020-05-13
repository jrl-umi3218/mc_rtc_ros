#include "Schema.h"

#include "SchemaArrayInput.h"

#include <mc_rtc/logging.h>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

#include <unordered_map>

namespace mc_rtc_rviz
{

namespace
{

std::string ref_path(const std::string & source, const std::string & ref_)
{
  auto ref = ref_.substr(3); // remove leading /..
  bfs::path ref_schema{source};
  ref_schema = bfs::canonical(ref_schema.parent_path() / ref);
  return ref_schema.string();
}

} // namespace

Schema::Schema(const std::string & file) : Schema(mc_rtc::Configuration{file}, file) {}

Schema::Schema(const std::string & file, const std::string & title) : Schema(mc_rtc::Configuration{file}, file, title)
{
}

Schema::Schema(const std::vector<Schema> & schemas)
{
  is_tuple_ = true;
  create_form = [schemas](QWidget * parent, const mc_rtc::Configuration & data) {
    std::vector<FormElement *> elems;
    for(const auto & s : schemas)
    {
      for(auto & el : s.create_form(parent, data))
      {
        elems.push_back(el);
      }
    }
    return elems;
  };
}

Schema::Schema(const mc_rtc::Configuration & s, const std::string & source, const std::string & title, bool required_in)
{
  if(s.has("allOf"))
  {
    auto schema = s;
    for(size_t i = 0; i < s("allOf").size(); ++i)
    {
      auto si = s("allOf")[i];
      if(si.has("$ref"))
      {
        schema.load(mc_rtc::Configuration{ref_path(source, si("$ref"))});
      }
      else
      {
        schema.load(si);
      }
    }
    schema.remove("allOf");
    init(schema, source, title, required_in);
  }
  else
  {
    init(s, source, title, required_in);
  }
}

void Schema::init(const mc_rtc::Configuration & s,
                  const std::string & source,
                  const std::string & title,
                  bool required_in)
{
  title_ = s("title", title.size() ? title : "No title property in " + source);
  auto required = s("required", std::vector<std::string>{});
  auto is_required = [&required](const std::string & label) {
    return std::find(required.begin(), required.end(), label) != required.end();
  };
  /** Resolve a $ref entry into a Schema */
  auto resolve_ref = [](const std::string & source, const std::string & ref_,
                        const std::string & title) -> const Schema & {
    auto ref_schema = ref_path(source, ref_);
    auto key = ref_schema + "::" + title;
    if(!store().count(key))
    {
      store()[key] = Schema{ref_schema, title};
    }
    return store().at(key);
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
                                     const mc_rtc::Configuration & schema) {
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
      Schema s(schema, source, k, required);
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
      static const std::unordered_map<std::string, std::string> surface_keys = {
          {"surface", "$robot"}, {"r1Surface", "$r1"}, {"r2Surface", "$r2"}};
      if(k == "robot" || k == "r1" || k == "r2")
      {
        create_form = [cf, k, required](QWidget * parent, const mc_rtc::Configuration & data) {
          auto v = cf(parent, data);
          v.emplace_back(new form::DataComboInput(parent, k, required, data, {"robots"}, false));
          return v;
        };
      }
      else if(k == "body")
      {
        create_form = [cf, k, required](QWidget * parent, const mc_rtc::Configuration & data) {
          auto v = cf(parent, data);
          v.emplace_back(new form::DataComboInput(parent, "body", required, data, {"bodies", "$robot"}, false));
          return v;
        };
      }
      else if(surface_keys.count(k))
      {
        const auto & robot = surface_keys.at(k);
        create_form = [cf, k, robot, required](QWidget * parent, const mc_rtc::Configuration & data) {
          auto v = cf(parent, data);
          v.emplace_back(new form::DataComboInput(parent, k, required, data, {"surfaces", robot}, false));
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
  auto handle_type_array = [this, &source](const std::string & k, bool required, const std::string & type, size_t min,
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
  auto handle_ref_array = [this, &source, &resolve_ref](const std::string & k, bool required, const std::string & ref,
                                                        size_t min, size_t max) {
    const auto & schema = resolve_ref(source, ref, "");
    auto cf = create_form;
    create_form = [cf, k, required, min, max, &schema](QWidget * parent, const mc_rtc::Configuration & data) {
      auto v = cf(parent, data);
      v.emplace_back(new form::SchemaArrayInput(parent, k, required, schema, data, min == max, min, max));
      return v;
    };
  };
  auto handle_array = [this, &source, &handle_type_array, &handle_ref_array](
                          const char * desc, const std::string & k, bool required, const mc_rtc::Configuration & c) {
    if(!c.has("items"))
    {
      LOG_WARNING(desc << k << " in " << source << " is an array but items are not specified")
    }
    auto items = c("items");
    int minItems = c("minItems", 0);
    int maxItems = c("maxItems", 256);
    if(items.has("type"))
    {
      handle_type_array(k, required, items("type"), minItems, maxItems);
    }
    else if(items.has("$ref"))
    {
      handle_ref_array(k, required, items("$ref"), minItems, maxItems);
    }
    else if(items.size())
    {
      std::vector<Schema> schemas;
      for(size_t i = 0; i < items.size(); ++i)
      {
        schemas.emplace_back(items[i], source, std::to_string(i), required);
      }
      Schema schema(schemas);
      auto cf = create_form;
      create_form = [cf, k, required, minItems, maxItems, schema](QWidget * parent,
                                                                  const mc_rtc::Configuration & data) {
        auto v = cf(parent, data);
        v.emplace_back(
            new form::SchemaArrayInput(parent, k, required, schema, data, minItems == maxItems, minItems, maxItems));
        return v;
      };
    }
    else
    {
      LOG_WARNING(desc << k << " in " << source << " is an array but items' type is not specified")
    }
  };
  auto handle_object = [&](const std::string & title, const mc_rtc::Configuration & schema) {
    if(!schema.has("properties"))
    {
      LOG_WARNING(title << " in " << source << " is an object without properties")
      return;
    }
    auto properties = schema("properties");
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
          handle_array("Property ", k, is_required(k), prop);
        }
        else
        {
          handle_type(k, is_required(k), type, prop);
        }
      }
      else if(prop.has("$ref"))
      {
        const auto & schema = resolve_ref(source, prop("$ref"), k);
        auto cf = create_form;
        bool required = is_required(k);
        create_form = [cf, k, required, &schema](QWidget * parent, const mc_rtc::Configuration & data) {
          auto v = cf(parent, data);
          if(schema.is_object())
          {
            v.emplace_back(new form::Form(parent, k, required, schema.create_form(parent, data)));
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
  };
  if(s.has("$ref"))
  {
    *this = resolve_ref(source, s("$ref"), title_);
    return;
  }
  if(!s.has("type"))
  {
    LOG_ERROR("No type entry for " << title_ << " in " << source)
  }
  std::string type = s("type");
  if(type == "object")
  {
    is_object_ = true;
    handle_object(title_, s);
  }
  else if(type == "array")
  {
    handle_array("", title_, required_in, s);
  }
  else
  {
    handle_type(title_, required_in, type, s);
  }
}

SchemaStore & Schema::store()
{
  static SchemaStore store;
  return store;
}

} // namespace mc_rtc_rviz
