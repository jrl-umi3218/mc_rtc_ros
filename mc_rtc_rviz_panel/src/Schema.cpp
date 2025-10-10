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
  bfs::path ref_schema{source};
  ref_schema = bfs::canonical(ref_schema.parent_path() / ref_);
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
  create_form = [schemas](QWidget * parent, const mc_rtc::Configuration & data)
  {
    std::vector<FormElement *> elems;
    for(const auto & s : schemas)
    {
      for(auto & el : s.create_form(parent, data)) { elems.push_back(el); }
    }
    return elems;
  };
}

mc_rtc::Configuration Schema::resolveAllOf(const mc_rtc::Configuration & s, const std::string & source) const
{
  if(s.has("allOf"))
  {
    auto schema = s;
    for(size_t i = 0; i < s("allOf").size(); ++i)
    {
      auto si = s("allOf")[i];
      if(si.has("$ref")) { schema.load(resolveAllOf(mc_rtc::Configuration{ref_path(source, si("$ref"))}, source)); }
      else if(si.has("allOf")) { schema.load(resolveAllOf(si, source)); }
      else
      {
        schema.load(si);
      }
    }
    schema.remove("allOf");
    return schema;
  }
  else
  {
    return s;
  }
}

Schema::Schema(const mc_rtc::Configuration & s, const std::string & source, const std::string & title, bool required_in)
{
  init(resolveAllOf(s, source), source, title, required_in);
}

void Schema::init(const mc_rtc::Configuration & s,
                  const std::string & source,
                  const std::string & title,
                  bool required_in)
{
  title_ = s("title", title.size() ? title : "No title property in " + source);
  auto required = s("required", std::vector<std::string>{});
  auto is_required = [&required](const std::string & label)
  { return std::find(required.begin(), required.end(), label) != required.end(); };
  /** Resolve a $ref entry into a Schema */
  auto resolve_ref = [](const std::string & sourceIn, const std::string & ref_,
                        const std::string & titleIn) -> const Schema &
  {
    auto ref_schema = ref_path(sourceIn, ref_);
    auto key = ref_schema + "::" + titleIn;
    if(!store().count(key)) { store()[key] = Schema{ref_schema, titleIn}; }
    return store().at(key);
  };
  /** Handle enum entries */
  auto handle_enum = [this](const std::string & k, bool requiredIn, const std::vector<std::string> & values)
  {
    auto cf = create_form;
    create_form = [cf, k, requiredIn, values](QWidget * parent, const mc_rtc::Configuration & data)
    {
      auto v = cf(parent, data);
      v.emplace_back(new form::ComboInput(parent, k, requiredIn, values, false, values.size() == 1 ? 0 : -1));
      if(values.size() == 1 && requiredIn) { v.back()->hidden(true); }
      return v;
    };
  };
  /** Handle const entries */
  auto handle_const = [this](const std::string & k, bool requiredIn, const std::string & value)
  {
    auto cf = create_form;
    create_form = [cf, k, requiredIn, value](QWidget * parent, const mc_rtc::Configuration & data)
    {
      auto v = cf(parent, data);
      v.emplace_back(new form::StringInput(parent, k, requiredIn, value, true));
      if(value.size() && requiredIn) { v.back()->hidden(true); }
      return v;
    };
  };
  /** Handle: boolean/integer/number/string */
  auto handle_type = [this, &source](const std::string & k, bool requiredIn, const std::string & type,
                                     const mc_rtc::Configuration & schema)
  {
    auto cf = create_form;
    if(type == "boolean")
    {
      create_form = [cf, k, requiredIn](QWidget * parent, const mc_rtc::Configuration & data)
      {
        auto v = cf(parent, data);
        v.emplace_back(new form::Checkbox(parent, k, requiredIn, true, false));
        return v;
      };
    }
    else if(type == "integer")
    {
      if(k == "robotIndex")
      {
        create_form = [cf, requiredIn](QWidget * parent, const mc_rtc::Configuration & data)
        {
          auto v = cf(parent, data);
          v.emplace_back(new form::DataComboInput(parent, "robot", requiredIn, data, {"robots"}, true, "robotIndex"));
          return v;
        };
        return;
      }
      create_form = [cf, k, requiredIn](QWidget * parent, const mc_rtc::Configuration & data)
      {
        auto v = cf(parent, data);
        v.emplace_back(new form::IntegerInput(parent, k, requiredIn, 0, false));
        return v;
      };
    }
    else if(type == "number")
    {
      create_form = [cf, k, requiredIn](QWidget * parent, const mc_rtc::Configuration & data)
      {
        auto v = cf(parent, data);
        v.emplace_back(new form::NumberInput(parent, k, requiredIn, 0, false));
        return v;
      };
    }
    else if(type == "object")
    {
      Schema next(schema, source, k, requiredIn);
      create_form = [cf, k, requiredIn, next](QWidget * parent, const mc_rtc::Configuration & data)
      {
        auto v = cf(parent, data);
        v.emplace_back(new form::Form(parent, k, requiredIn, next.create_form(parent, data)));
        return v;
      };
    }
    else
    {
      if(type != "string")
      {
        mc_rtc::log::warning("Property {} in {} has unknown or missing type (value: {}), treating as string", k, source,
                             type);
      }
      static const std::unordered_map<std::string, std::string> surface_keys = {
          {"surface", "$robot"}, {"r1Surface", "$r1"}, {"r2Surface", "$r2"}};
      if(k == "robot" || k == "r1" || k == "r2")
      {
        create_form = [cf, k, requiredIn](QWidget * parent, const mc_rtc::Configuration & data)
        {
          auto v = cf(parent, data);
          v.emplace_back(new form::DataComboInput(parent, k, requiredIn, data, {"robots"}, false));
          return v;
        };
      }
      else if(k == "body")
      {
        create_form = [cf, k, requiredIn](QWidget * parent, const mc_rtc::Configuration & data)
        {
          auto v = cf(parent, data);
          v.emplace_back(new form::DataComboInput(parent, "body", requiredIn, data, {"bodies", "$robot"}, false));
          return v;
        };
      }
      else if(surface_keys.count(k))
      {
        const auto & robot = surface_keys.at(k);
        create_form = [cf, k, robot, requiredIn](QWidget * parent, const mc_rtc::Configuration & data)
        {
          auto v = cf(parent, data);
          v.emplace_back(new form::DataComboInput(parent, k, requiredIn, data, {"surfaces", robot}, false));
          return v;
        };
      }
      else if(k == "frame")
      {
        create_form = [cf, k, requiredIn](QWidget * parent, const mc_rtc::Configuration & data)
        {
          auto v = cf(parent, data);
          v.emplace_back(new form::DataComboInput(parent, "frame", requiredIn, data, {"frames", "$robot"}, false));
          return v;
        };
      }
      else
      {
        create_form = [cf, k, requiredIn](QWidget * parent, const mc_rtc::Configuration & data)
        {
          auto v = cf(parent, data);
          v.emplace_back(new form::StringInput(parent, k, requiredIn, "", false));
          return v;
        };
      }
    }
  };
  /** Handle an array */
  auto handle_type_array =
      [this, &source](const std::string & k, bool requiredIn, const std::string & type, size_t min, size_t max)
  {
    auto cf = create_form;
    if(type == "integer")
    {
      create_form = [cf, k, requiredIn, min, max](QWidget * parent, const mc_rtc::Configuration & data)
      {
        auto v = cf(parent, data);
        v.emplace_back(new form::IntegerArrayInput(parent, k, requiredIn, min == max, static_cast<int>(min),
                                                   static_cast<int>(max)));
        return v;
      };
    }
    else if(type == "number")
    {
      create_form = [cf, k, requiredIn, min, max](QWidget * parent, const mc_rtc::Configuration & data)
      {
        auto v = cf(parent, data);
        v.emplace_back(new form::NumberArrayInput(parent, k, requiredIn, min == max, static_cast<int>(min),
                                                  static_cast<int>(max)));
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
        mc_rtc::log::warning(
            "Property {} in {} has unknown or missing array items' type (value: {}), treating as string", k, source,
            type);
      }
      create_form = [cf, k, requiredIn, min, max](QWidget * parent, const mc_rtc::Configuration & data)
      {
        auto v = cf(parent, data);
        v.emplace_back(new form::StringArrayInput(parent, k, requiredIn, min == max, static_cast<int>(min),
                                                  static_cast<int>(max)));
        return v;
      };
    }
  };
  auto handle_ref_array = [this, &source, &resolve_ref](const std::string & k, bool requiredIn, const std::string & ref,
                                                        size_t min, size_t max)
  {
    const auto & schema = resolve_ref(source, ref, "");
    auto cf = create_form;
    create_form = [cf, k, requiredIn, min, max, &schema](QWidget * parent, const mc_rtc::Configuration & data)
    {
      auto v = cf(parent, data);
      v.emplace_back(new form::SchemaArrayInput(parent, k, requiredIn, schema, data, min == max, static_cast<int>(min),
                                                static_cast<int>(max)));
      return v;
    };
  };
  auto handle_array = [this, &source, &handle_type_array, &handle_ref_array](
                          const char * desc, const std::string & k, bool requiredIn, const mc_rtc::Configuration & c)
  {
    if(!c.has("items")) { mc_rtc::log::warning("{}{} in {} is an array but items are not specified", desc, k, source); }
    auto items = c("items");
    size_t minItems = c("minItems", size_t{0});
    size_t maxItems = c("maxItems", size_t{256});
    if(items.has("type")) { handle_type_array(k, requiredIn, items("type"), minItems, maxItems); }
    else if(items.has("$ref")) { handle_ref_array(k, requiredIn, items("$ref"), minItems, maxItems); }
    else if(items.size())
    {
      std::vector<Schema> schemas;
      for(size_t i = 0; i < items.size(); ++i)
      {
        schemas.emplace_back(items[i], source, std::to_string(i), requiredIn);
      }
      Schema schema(schemas);
      auto cf = create_form;
      create_form =
          [cf, k, requiredIn, minItems, maxItems, schema](QWidget * parent, const mc_rtc::Configuration & data)
      {
        auto v = cf(parent, data);
        v.emplace_back(new form::SchemaArrayInput(parent, k, requiredIn, schema, data, minItems == maxItems,
                                                  static_cast<int>(minItems), static_cast<int>(maxItems)));
        return v;
      };
    }
    else
    {
      mc_rtc::log::warning("{}{} in {} is an array but items' type is not specified", desc, k, source);
    }
  };
  auto handle_object = [&](const std::string & titleIn, const mc_rtc::Configuration & schemaIn, bool requiredInParent)
  {
    if(!schemaIn.has("properties"))
    {
      mc_rtc::log::warning("{} in {} is an object without properties", titleIn, source);
      return;
    }
    auto properties = schemaIn("properties");
    for(const auto & k : properties.keys())
    {
      auto prop = properties(k);
      if(prop.has("enum")) { handle_enum(k, is_required(k), prop("enum")); }
      else if(prop.has("const")) { handle_const(k, is_required(k), prop("const")); }
      else if(prop.has("type"))
      {
        std::string type = prop("type");
        if(type == "array") { handle_array("Property ", k, is_required(k), prop); }
        else
        {
          handle_type(k, is_required(k), type, prop);
        }
      }
      else if(prop.has("$ref"))
      {
        const auto & schema = resolve_ref(source, prop("$ref"), k);
        auto cf = create_form;
        bool requiredIn = is_required(k);
        create_form =
            [cf, k, requiredIn, requiredInParent, &schema](QWidget * parent, const mc_rtc::Configuration & data)
        {
          auto v = cf(parent, data);
          if(schema.is_object())
          {
            v.emplace_back(new form::Form(parent, k, requiredInParent, schema.create_form(parent, data)));
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
        mc_rtc::log::error("Cannot handle property {} in {}: {}", k, source, prop.dump());
      }
    }
  };
  if(s.has("$ref"))
  {
    *this = resolve_ref(source, s("$ref"), title_);
    return;
  }
  if(!s.has("type")) { mc_rtc::log::error("No type entry for {} in {}", title, source); }
  std::string type = s("type");
  if(type == "object")
  {
    is_object_ = true;
    handle_object(title_, s, required_in);
  }
  else if(type == "array") { handle_array("", title_, required_in, s); }
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
