/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "FormElement.h"

namespace mc_rtc_rviz
{

FormElement::FormElement(QWidget * parent, const std::string & name, bool required)
: QWidget(parent), required_(required), name_(name)
{
}

bool FormElement::eventFilter(QObject * o, QEvent * e)
{
  if(e->type() == QEvent::Wheel)
  {
    if(qobject_cast<QComboBox *>(o) != nullptr)
    {
      e->ignore();
      return true;
    }
  }
  return QWidget::eventFilter(o, e);
}

bool FormElement::can_fill(std::string & msg)
{
  if(ready())
  {
    return true;
  }
  else if(required())
  {
    msg += name() + " must be completed";
    return false;
  }
  return true;
}

namespace form
{

namespace details
{

template<typename T>
mc_rtc::Configuration serialize(const T & data)
{
  mc_rtc::Configuration out;
  out.add("data", data);
  return out("data");
}

} // namespace details

Checkbox::Checkbox(QWidget * parent, const std::string & name, bool required, bool def)
: FormElement(parent, name, required), def_(def)
{
  auto layout = new QVBoxLayout(this);
  cbox_ = new QCheckBox(this);
  cbox_->setChecked(def);
  connect(cbox_, SIGNAL(toggled(bool)), this, SLOT(toggled(bool)));
  layout->addWidget(cbox_);
}

bool Checkbox::changed(bool required, bool def)
{
  return changed_(required) || def_ != def;
}

void Checkbox::toggled(bool)
{
  ready_ = true;
}

mc_rtc::Configuration Checkbox::serialize() const
{
  return details::serialize(cbox_->isChecked());
}

CommonInput::CommonInput(QWidget * parent, const std::string & name, bool required)
: FormElement(parent, name, required)
{
  auto layout = new QVBoxLayout(this);
  edit_ = new QLineEdit(this);
  layout->addWidget(edit_);
}

void CommonInput::textChanged(const QString &)
{
  ready_ = true;
}

IntegerInput::IntegerInput(QWidget * parent, const std::string & name, bool required, int def)
: CommonInput(parent, name, required), def_(def)
{
  auto validator = new QIntValidator(this);
  validator->setLocale(QLocale::C);
  edit_->setValidator(validator);
  edit_->setText(QString::number(def));
  connect(edit_, SIGNAL(textChanged(const QString &)), this, SLOT(textChanged(const QString &)));
}

bool IntegerInput::changed(bool required, int def)
{
  return changed_(required) || def_ != def;
}

mc_rtc::Configuration IntegerInput::serialize() const
{
  return details::serialize(edit_->text().toInt());
}

NumberInput::NumberInput(QWidget * parent, const std::string & name, bool required, double def)
: CommonInput(parent, name, required), def_(def)
{
  auto validator = new QDoubleValidator(this);
  validator->setLocale(QLocale::C);
  edit_->setValidator(validator);
  edit_->setText(QString::number(def));
  connect(edit_, SIGNAL(textChanged(const QString &)), this, SLOT(textChanged(const QString &)));
}

bool NumberInput::changed(bool required, double def)
{
  return changed_(required) || def_ != def;
}

mc_rtc::Configuration NumberInput::serialize() const
{
  return details::serialize(edit_->text().toDouble());
}

StringInput::StringInput(QWidget * parent, const std::string & name, bool required, const std::string & def)
: CommonInput(parent, name, required), def_(def)
{
  edit_->setText(def.c_str());
  connect(edit_, SIGNAL(textChanged(const QString &)), this, SLOT(textChanged(const QString &)));
}

bool StringInput::changed(bool required, const std::string & def)
{
  return changed_(required) || def_ != def;
}

mc_rtc::Configuration StringInput::serialize() const
{
  return details::serialize(edit_->text().toStdString());
}

CommonArrayInput::CommonArrayInput(QWidget * parent, const std::string & name, bool required)
: FormElement(parent, name, required)
{
}

void CommonArrayInput::textChanged(const QString &)
{
  ready_ = true;
}

template<>
void ArrayInput<int>::add_validator(QLineEdit * edit)
{
  auto validator = new QIntValidator(this);
  validator->setLocale(QLocale::C);
  edit->setValidator(validator);
}
template<>
void ArrayInput<double>::add_validator(QLineEdit * edit)
{
  auto validator = new QDoubleValidator();
  validator->setLocale(QLocale::C);
  edit->setValidator(validator);
}
template<>
void ArrayInput<int>::data2edit(const int & value, QLineEdit * edit)
{
  edit->setText(QString::number(value));
}
template<>
int ArrayInput<int>::edit2data(QLineEdit * edit) const
{
  return edit->text().toInt();
}
template<>
void ArrayInput<double>::data2edit(const double & value, QLineEdit * edit)
{
  edit->setText(QString::number(value));
}
template<>
double ArrayInput<double>::edit2data(QLineEdit * edit) const
{
  return edit->text().toDouble();
}
template<>
void ArrayInput<std::string>::add_validator(QLineEdit *)
{
}
template<>
void ArrayInput<std::string>::data2edit(const std::string & value, QLineEdit * edit)
{
  edit->setText(value.c_str());
}
template<>
std::string ArrayInput<std::string>::edit2data(QLineEdit * edit) const
{
  return edit->text().toStdString();
}

NumberArrayInput::NumberArrayInput(QWidget * parent,
                                   const std::string & name,
                                   bool required,
                                   const Eigen::VectorXd & def,
                                   bool fixed_size)
: ArrayInput<double>(parent, name, required, fixed_size), def_(def)
{
  for(int i = 0; i < def.size(); ++i)
  {
    add_edit(def(i));
  }
}

NumberArrayInput::NumberArrayInput(QWidget * parent,
                                   const std::string & name,
                                   bool required,
                                   bool fixed_size,
                                   int min_size,
                                   int max_size)
: ArrayInput<double>(parent, name, required, fixed_size, min_size, max_size)
{
}

bool NumberArrayInput::changed(bool required, const Eigen::VectorXd & def, bool fixed_size)
{
  return changed_(required) || def_ != def || fixed_size_ != fixed_size;
}

ComboInput::ComboInput(QWidget * parent,
                       const std::string & name,
                       bool required,
                       const std::vector<std::string> & values,
                       bool send_index)
: FormElement(parent, name, required), values_(values), send_index_(send_index)
{
  auto layout = new QVBoxLayout(this);
  combo_ = new QComboBox(this);
  combo_->installEventFilter(this);
  combo_->setFocusPolicy(Qt::StrongFocus);
  layout->addWidget(combo_);
  for(const auto & v : values)
  {
    combo_->addItem(v.c_str());
  }
  combo_->setCurrentIndex(-1);
  if(values.size() == 1 && required)
  {
    combo_->setCurrentIndex(0);
    ready_ = true;
  }
  connect(combo_, SIGNAL(currentIndexChanged(int)), this, SLOT(currentIndexChanged(int)));
}

bool ComboInput::changed(bool required, const std::vector<std::string> & values, bool send_index)
{
  return changed_(required) || values_ != values || send_index_ != send_index;
}

void ComboInput::currentIndexChanged(int idx)
{
  ready_ = idx != -1;
}

mc_rtc::Configuration ComboInput::serialize() const
{
  if(send_index_)
  {
    return details::serialize(combo_->currentIndex());
  }
  else
  {
    return details::serialize(combo_->currentText().toStdString());
  }
}

DataComboInput::DataComboInput(QWidget * parent,
                               const std::string & name,
                               bool required,
                               const mc_rtc::Configuration & data,
                               const std::vector<std::string> & ref,
                               bool send_index,
                               const std::string & rename)
: FormElement(parent, name, required), send_index_(send_index), data_(data), ref_(ref), resolved_ref_(ref),
  rename_(rename.size() ? rename : name)
{
  auto layout = new QVBoxLayout(this);
  combo_ = new QComboBox(this);
  combo_->installEventFilter(this);
  combo_->setFocusPolicy(Qt::StrongFocus);
  layout->addWidget(combo_);
  if(resolve_ref())
  {
    update_values();
  }
  connect(combo_, SIGNAL(currentIndexChanged(int)), this, SLOT(currentIndexChanged(int)));
}

bool DataComboInput::changed(bool required,
                             const mc_rtc::Configuration &,
                             const std::vector<std::string> & ref,
                             bool send_index)
{
  bool c = changed_(required) || ref_ != ref || send_index_ != send_index;
  if(c)
  {
    return c;
  }
  if(resolve_ref())
  {
    update_values();
  }
  return c;
}

void DataComboInput::currentIndexChanged(int idx)
{
  ready_ = idx != -1;
}

void DataComboInput::update_dependencies(FormElement * other)
{
  for(const auto & k : ref_)
  {
    if(k.size() && k[0] == '$')
    {
      auto need = k.substr(1, k.size() - 1);
      if(other->name() == need && !connected_.count(need))
      {
        auto combo = static_cast<QComboBox *>(other->layout()->itemAt(0)->widget());
        connect(combo, SIGNAL(currentIndexChanged(const QString &)), this, SLOT(currentIndexChanged(const QString &)));
        connected_.insert(need);
        if(combo->currentIndex() != -1)
        {
          update_field(other->name(), combo->currentText().toStdString());
        }
      }
    }
  }
}

void DataComboInput::currentIndexChanged(const QString & text)
{
  auto source = qobject_cast<FormElement *>(sender()->parent());
  assert(source);
  update_field(source->name(), text.toStdString());
}

mc_rtc::Configuration DataComboInput::serialize() const
{
  if(send_index_)
  {
    return details::serialize(combo_->currentIndex());
  }
  else
  {
    return details::serialize(combo_->currentText().toStdString());
  }
}

bool DataComboInput::resolve_ref()
{
  for(const auto & k : resolved_ref_)
  {
    if(k.size() == 0 || k[0] == '$')
    {
      return false;
    }
  }
  return true;
}

void DataComboInput::update_field(const std::string & name, const std::string & value)
{
  for(size_t i = 0; i < ref_.size(); ++i)
  {
    if(ref_[i].size() && ref_[i][0] == '$')
    {
      auto k = ref_[i].substr(1, ref_[i].size() - 1);
      if(k == name)
      {
        resolved_ref_[i] = value;
      }
    }
  }
  if(resolve_ref())
  {
    update_values();
  }
}

void DataComboInput::update_values()
{
  auto data = data_;
  for(const auto & k : resolved_ref_)
  {
    data = data(k, mc_rtc::Configuration{});
  }
  auto values = data.size() ? data : std::vector<std::string>{};
  if(values_ != values)
  {
    auto selected = combo_->currentText().toStdString();
    int idx = -1;
    values_ = values;
    combo_->clear();
    for(size_t i = 0; i < values.size(); ++i)
    {
      const auto & v = values[i];
      if(v == selected)
      {
        idx = static_cast<int>(i);
      }
      combo_->addItem(v.c_str());
    }
    if(idx != -1)
    {
      combo_->setCurrentIndex(idx);
    }
    else if(values.size())
    {
      combo_->setCurrentIndex(0);
    }
  }
}

Form::Form(QWidget * parent,
           const std::string & name,
           bool required,
           const std::vector<FormElement *> elements,
           bool checkable,
           bool use_group_name)
: FormElement(parent, name, required), elements_(elements)
{
  spanning_ = true;
  show_name_ = false;
  auto title = required ? name + "*" : name;
  auto m_layout = new QVBoxLayout(this);
  group_ = new QGroupBox(use_group_name ? title.c_str() : "", parent);
  group_->setStyleSheet("QGroupBox {border: 1px solid gray;border-radius: 9px;margin-top: 0.5em;} QGroupBox::title { "
                        "subcontrol-origin: margin; left: 10px; margin-bottom: 2em; padding: 0 3px 0 3px;}");
  auto layout = new QFormLayout(group_);
  for(auto & el : elements_)
  {
    for(auto & other : elements_)
    {
      el->update_dependencies(other);
    }
    auto el_name = el->required() ? el->name() + "*" : el->name();
    if(!el->spanning())
    {
      layout->addRow(el_name.c_str(), el);
    }
    else
    {
      if(el->show_name())
      {
        layout->addRow(new QLabel(el_name.c_str(), this));
      }
      layout->addRow(el);
    }
  }
  m_layout->addWidget(group_);
  group_->setCheckable(checkable);
  if(checkable)
  {
    group_->setChecked(true);
    connect(group_, SIGNAL(toggled(bool)), this, SIGNAL(toggled(bool)));
  }
}

bool Form::ready() const
{
  bool ok = false;
  for(const auto & el : elements_)
  {
    if(el->ready())
    {
      ok = true;
    }
    if(el->required() && !el->ready())
    {
      return false;
    }
  }
  return ok;
}

mc_rtc::Configuration Form::serialize() const
{
  return serialize(false);
}

mc_rtc::Configuration Form::serialize(bool asTuple) const
{
  mc_rtc::Configuration out;
  if(asTuple)
  {
    out = out.array("data", elements_.size());
  }
  for(auto & el : elements_)
  {
    if(el->ready())
    {
      if(asTuple)
      {
        out.push(el->serialize());
      }
      else
      {
        out.add(el->name(), el->serialize());
      }
    }
  }
  return out;
}

void Form::rejectUncheck()
{
  group_->setChecked(true);
}

} // namespace form

} // namespace mc_rtc_rviz
