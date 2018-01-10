#include "SchemaForm.h"

#include <mc_rtc/Configuration.h>

#include <QCheckBox>
#include <QComboBox>
#include <QFileInfo>
#include <QIntValidator>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>

#include <iostream>

template<typename T>
QValidator * makeValidator() { return nullptr; }
template<>
QValidator * makeValidator<int>()
{
  return new QIntValidator();
}
template<>
QValidator * makeValidator<double>()
{
  return new QDoubleValidator();
}

template<typename T>
struct send_ret { using t = std::string; };
template<> struct send_ret<double> { using t = double; };
template<> struct send_ret<int> { using t = int; };

template<typename T>
typename send_ret<T>::t send(const QLineEdit & le)
{
  return le.text().toStdString();
}
template<>
double send<double>(const QLineEdit & le)
{
  return le.text().toDouble();
}
template<>
int send<int>(const QLineEdit & le)
{
  return le.text().toInt();
}


template<typename T>
struct ArrayInput : public QWidget
{
  ArrayInput(size_t min_items, size_t max_items)
  : min_items_(min_items),
    max_items_(max_items)
  {
    QLayout * layout = nullptr;
    validator_ = makeValidator<T>();
    if(min_items == max_items)
    {
      /** Fixed size */
      if(max_items < 9)
      {
        layout = new QHBoxLayout();
        for(size_t i = 0; i < max_items; ++i)
        {
          les_.push_back(new QLineEdit());
          les_.back()->setValidator(validator_);
          layout->addWidget(les_.back());
        }
      }
      else
      {
        size_t cols = static_cast<size_t>(floor(sqrt(max_items)));
        layout = new QVBoxLayout();
        size_t i = 0;
        for(i = 0; i < max_items/cols; ++i)
        {
          auto hlayout = new QHBoxLayout();
          for(size_t j = 0; j < cols; ++j)
          {
            les_.push_back(new QLineEdit());
            les_.back()->setValidator(validator_);
            hlayout->addWidget(les_.back());
          }
          layout->addItem(hlayout);
        }
        size_t remain = i*cols;
        if(max_items - remain > 0)
        {
          auto hlayout = new QHBoxLayout();
          for(i = remain; i < max_items; ++i)
          {
            les_.push_back(new QLineEdit());
            les_.back()->setValidator(validator_);
            hlayout->QLayout::addWidget(les_.back());
          }
          layout->addItem(hlayout);
        }
      }
    }
    else
    {
      layout = new QVBoxLayout();
      for(size_t i = 0; i < min_items; ++i)
      {
        les_.push_back(new QLineEdit());
        les_.back()->setValidator(makeValidator<T>());
        layout->addWidget(les_.back());
      }
      more = new QPushButton("+");
      connect(more, &QPushButton::released,
        [this]()
        {
        std::cout << "+ clicked" << std::endl;
        });
      layout->addWidget(more);
    }
    assert(layout);
    setLayout(layout);
  }

  bool ready()
  {
    if(les_.size() == 0) { return false; }
    for(const auto & le : les_)
    {
      if(le->text().size() == 0) { return false; }
    }
    return true;
  }

  void send(mc_rtc::Configuration & out)
  {
    for(const auto & le : les_)
    {
      out.push(::send<T>(*le));
    }
  }

  int min_items_;
  int max_items_;
  QValidator * validator_ = nullptr;
  std::vector<QLineEdit*> les_;
  std::vector<QPushButton*> le_remove_;
  QPushButton * more = nullptr;
};

struct SchemaFormItem
{
  QWidget * widget = nullptr; // Can be nullptr (hidden item)
  bool spanning = false;
  std::string name;
  std::string title;
  std::function<bool()> ready = [](){ return true; };
  std::function<void(mc_rtc::Configuration&)> send = [](mc_rtc::Configuration&){};

  SchemaFormItem(std::string name) : name(std::move(name)) {}

  static SchemaFormItem * make(const std::string & name,
                             const mc_rtc::Configuration & data,
                             bool is_required,
                             const std::string & schema,
                             QFormLayout * layout,
                             SchemaFormItem * ret = nullptr);
};

SchemaFormItem * SchemaFormItem::make(const std::string & name,
                                      const mc_rtc::Configuration & data,
                                      bool is_required,
                                      const std::string & schema,
                                      QFormLayout * layout,
                                      SchemaFormItem * ret_)
{
  SchemaFormItem * ret = ret_ ? ret_ : new SchemaFormItem{name};
  if(data.has("enum"))
  {
    /** Handle enum fields, display possible values if enums size is > 1, otherwise have an hidden field */
    std::vector<std::string> enums = data("enum");
    if(enums.size() != 1)
    {
      auto cb = new QComboBox();
      for(const auto & s : enums)
      {
        cb->addItem(s.c_str());
      }
      ret->widget = cb;
      ret->send = [name,cb](mc_rtc::Configuration & out)
      {
        out.add(name, cb->currentText().toStdString());
      };
    }
    else
    {
      if(enums.size() == 0)
      {
        throw(std::invalid_argument("enum field cannot be empty list"));
      }
      ret->send = [name,enums](mc_rtc::Configuration & out)
      {
        out.add(name, enums[0]);
      };
    }
  }
  else if(data.has("type"))
  {
    std::string type = data("type");
    if(type == "boolean")
    {
      auto cb = new QCheckBox();
      ret->widget = cb;
      ret->send = [name,cb](mc_rtc::Configuration & out)
      {
        out.add(name, cb->isChecked());
      };
    }
    else if(type == "number" || type == "integer" || type == "string")
    {
      auto le = new QLineEdit();
      QValidator * validator = nullptr;
      if(type == "number")
      {
        double min = data("minimum", -std::numeric_limits<double>::infinity());
        double max = data("maximum", std::numeric_limits<double>::infinity());
        validator = new QDoubleValidator(min, max, 100);
        ret->send = [name,le](mc_rtc::Configuration & out)
        {
          if(le->text().size())
          {
            out.add(name, le->text().toDouble());
          }
        };
      }
      else if(type == "integer")
      {
        int min = data("minimum", std::numeric_limits<int>::lowest());
        int max = data("maximum", std::numeric_limits<int>::max());
        validator = new QIntValidator(min, max);
        ret->send = [name,le](mc_rtc::Configuration & out)
        {
          if(le->text().size())
          {
            out.add(name, le->text().toInt());
          }
        };
      }
      else
      {
        ret->send = [name,le](mc_rtc::Configuration & out)
        {
          if(le->text().size())
          {
            out.add(name, le->text().toStdString());
          }
        };
      }
      le->setValidator(validator);
      ret->widget = le;
      if(is_required)
      {
        ret->ready = [le]()
        {
          return le->text().size() != 0;
        };
      }
    }
    else if(type == "array")
    {
      ret->spanning = true;
      if(data.has("title"))
      {
        ret->title = static_cast<std::string>(data("title"));
      }
      if(data.has("items"))
      {
        auto items = data("items");
        if(items.has("type"))
        {
          std::string items_type = items("type");
#define IMPL_IT(T) \
          unsigned int minItems = data("minItems", 0); \
          unsigned int maxItems = data("maxItems", std::numeric_limits<unsigned int>::max()); \
          auto ai = new ArrayInput<T>(minItems, maxItems); \
          ret->widget = ai; \
          if(is_required) { ret->ready = [ai]() { return ai->ready(); }; } \
          ret->send = [name,ai](mc_rtc::Configuration & out) \
          { \
            if(ai->ready()) \
            { \
              auto arr = out.array(name); \
              ai->send(arr); \
            } \
          };
          if(items_type == "integer") { IMPL_IT(int) }
          else if(items_type == "number") { IMPL_IT(double) }
          else if(items_type == "string") { IMPL_IT(std::string) }
          else
          {
            std::cerr << "Array " << name << " has items' type " << items_type << " that cannot be handled in schema " << schema << std::endl;
          }
        }
        else if(items.has("$ref"))
        {
          /**FIXME Not sure how to handle this */
          std::cerr << "Array " << name << " has items' $ref " << items("$ref") << " that cannot be handled in schema " << schema << std::endl;
        }
        else
        {
          std::cerr << "Array " << name << " has items without type or $ref specification in schema " << schema << std::endl;
        }
      }
      else
      {
        std::cerr << "Array " << name << " has no items specification in schema " << schema << std::endl;
      }
    }
    else if(type == "object")
    {
      ret->spanning = true;
      auto w = new SchemaForm(schema); // FIXME We don't need to open the schema again
      ret->widget = w;
      ret->title = w->title;
      if(is_required)
      {
        ret->ready = [w](){ return w->ready().size() == 0; };
      }
      ret->send = [name,w](mc_rtc::Configuration & out)
      {
        out.add(name, w->send());
      };
    }
    else
    {
      std::cerr << "Cannot handle type in JSON schema: " << type << std::endl;
    }
  }
  else if(data.has("$ref"))
  {
    auto schema_dir = QFileInfo{schema.c_str()}.path().toStdString();
    std::string ref = data("$ref");
    ref = ref.substr(strlen("/.."), ref.size() - strlen("/.."));
    std::string ref_schema = schema_dir + ref;
    ret = SchemaFormItem::make(name, mc_rtc::Configuration{ref_schema}, is_required, ref_schema, layout, ret);
    return ret;
  }
  else
  {
    std::cerr << "Property " << name << " in " << schema << " does not have enum, type or $ref property" << std::endl;
  }
  std::string display_name = name;
  if(is_required) { display_name += " * "; }
  if(ret->title.size()) { display_name += " (" + ret->title + ")"; }
  if(ret->widget)
  {
    if(ret->spanning)
    {
      auto w = new QWidget();
      auto l = new QVBoxLayout();
      l->addWidget(new QLabel(display_name.c_str()));
      l->addWidget(ret->widget);
      w->setObjectName("SPANNINGFORMITEM");
      w->setStyleSheet("#SPANNINGFORMITEM { border: 1px solid black; border-radius: 15px; }");
      w->setLayout(l);
      layout->setWidget(layout->rowCount(), QFormLayout::ItemRole::SpanningRole, w);
    }
    else
    {
      layout->addRow((display_name).c_str(), ret->widget);
    }
  }
  return ret;
}

SchemaForm::SchemaForm(const std::string & schema)
{
  mc_rtc::Configuration config(schema);
  title = static_cast<std::string>(config("title"));
  setLayout(layout);

  auto required = config("required", std::vector<std::string>{});
  auto is_required = [&required](const std::string & in)
  {
    return std::find(required.begin(), required.end(), in) != required.end();
  };
  auto properties = config("properties", std::map<std::string, mc_rtc::Configuration>{});
  for(const auto & p : properties)
  {
    items_.emplace_back(SchemaFormItem::make(p.first, p.second, is_required(p.first), schema, layout));
  }
}

std::string SchemaForm::ready()
{
  std::string out;
  for(auto & i : items_)
  {
    if(!i->ready())
    {
      out += "- " + i->name + " must be filled!\n";
    }
  }
  if(out.size()) { return out.substr(0, out.size() - 1); }
  return out;
}

mc_rtc::Configuration SchemaForm::send()
{
  mc_rtc::Configuration ret;
  for(auto & i : items_)
  {
    i->send(ret);
  }
  return ret;
}
