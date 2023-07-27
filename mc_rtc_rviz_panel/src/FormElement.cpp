/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "FormElement.h"

#include "FormElementContainer.h"

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
  if(ready()) { return true; }
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

Checkbox::Checkbox(QWidget * parent, const std::string & name, bool required, bool def, bool user_def)
: FormElement(parent, name, required), def_(def), user_def_(user_def)
{
  auto layout = new QVBoxLayout(this);
  cbox_ = new QCheckBox(this);
  layout->addWidget(cbox_);
  reset();
}

void Checkbox::changed(bool required, bool def, bool user_def)
{
  if(cbox_->hasFocus()) { return; }
  changed_(required);
  def_ = def;
  user_def_ = user_def;
  reset();
}

void Checkbox::toggled(bool)
{
  ready_ = true;
  locked_ = true;
}

void Checkbox::reset()
{
  cbox_->disconnect();
  if(user_def_) { cbox_->setChecked(def_); }
  else
  {
    cbox_->setTristate(true);
    cbox_->setCheckState(Qt::CheckState::PartiallyChecked);
  }
  ready_ = user_def_;
  connect(cbox_, SIGNAL(toggled(bool)), this, SLOT(toggled(bool)));
  connect(cbox_, SIGNAL(stateChanged(int)), this, SLOT(stateChanged(int)));
}

void Checkbox::stateChanged(int s)
{
  if(s != Qt::CheckState::PartiallyChecked)
  {
    ready_ = true;
    locked_ = true;
    cbox_->setTristate(false);
  }
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
  locked_ = true;
  ready_ = true;
}

IntegerInput::IntegerInput(QWidget * parent, const std::string & name, bool required, int def, bool user_def)
: CommonInput(parent, name, required), def_(def), user_def_(user_def)
{
  auto validator = new QIntValidator(this);
  validator->setLocale(QLocale::C);
  edit_->setValidator(validator);
  reset();
}

void IntegerInput::changed(bool required, int def, bool user_def)
{
  if(edit_->hasFocus()) { return; }
  changed_(required);
  def_ = def;
  user_def_ = user_def;
  reset();
}

mc_rtc::Configuration IntegerInput::serialize() const
{
  return details::serialize(edit_->text().toInt());
}

void IntegerInput::reset()
{
  edit_->disconnect();
  if(user_def_) { edit_->setText(QString::number(def_)); }
  else { edit_->setText(""); }
  ready_ = user_def_;
  connect(edit_, SIGNAL(textChanged(const QString &)), this, SLOT(textChanged(const QString &)));
}

NumberInput::NumberInput(QWidget * parent, const std::string & name, bool required, double def, bool user_def)
: CommonInput(parent, name, required), def_(def), user_def_(user_def)
{
  auto validator = new QDoubleValidator(this);
  validator->setLocale(QLocale::C);
  edit_->setValidator(validator);
  reset();
}

void NumberInput::changed(bool required, double def, bool user_def)
{
  if(edit_->hasFocus()) { return; }
  changed_(required);
  def_ = def;
  user_def_ = user_def;
  reset();
}

mc_rtc::Configuration NumberInput::serialize() const
{
  return details::serialize(edit_->text().toDouble());
}

void NumberInput::reset()
{
  edit_->disconnect();
  if(user_def_) { edit_->setText(QString::number(def_)); }
  else { edit_->setText(""); }
  ready_ = user_def_;
  connect(edit_, SIGNAL(textChanged(const QString &)), this, SLOT(textChanged(const QString &)));
}

StringInput::StringInput(QWidget * parent,
                         const std::string & name,
                         bool required,
                         const std::string & def,
                         bool user_def)
: CommonInput(parent, name, required), def_(def), user_def_(user_def)
{
  reset();
}

void StringInput::changed(bool required, const std::string & def, bool user_def)
{
  if(edit_->hasFocus()) { return; }
  changed_(required);
  def_ = def;
  user_def_ = user_def;
  reset();
}

void StringInput::reset()
{
  edit_->disconnect();
  if(user_def_) { edit_->setText(def_.c_str()); }
  else { edit_->setText(""); }
  ready_ = user_def_;
  connect(edit_, SIGNAL(textChanged(const QString &)), this, SLOT(textChanged(const QString &)));
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
  locked_ = true;
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
                                   bool fixed_size,
                                   bool user_def)
: ArrayInput<double>(parent, name, required, fixed_size), def_(def), user_def_(user_def)
{
  if(fixed_size) { updateStride(static_cast<size_t>(def.size())); }
  reset();
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

void NumberArrayInput::changed(bool required, const Eigen::VectorXd & def, bool /*fixed_size*/, bool user_def)
{
  for(const auto & e : edits_)
  {
    if(e->hasFocus()) { return; }
  }
  changed_(required);
  bool should_reset = def_.size() != def.size();
  def_ = def;
  user_def_ = user_def;
  fixed_size_ = fixed_size_;
  if(should_reset) { reset(); }
  size_t i = 0;
  for(auto & e : edits_)
  {
    e->disconnect();
    data2edit(def(static_cast<Eigen::DenseIndex>(i)), e);
    connect(e, SIGNAL(textChanged(const QString &)), this, SLOT(textChanged(const QString &)));
    i += 1;
  }
}

void NumberArrayInput::reset()
{
  ArrayInput<double>::reset();
  for(int i = 0; i < def_.size(); ++i) { add_edit(def_(i)); }
  ready_ = user_def_;
}

ComboInput::ComboInput(QWidget * parent,
                       const std::string & name,
                       bool required,
                       const std::vector<std::string> & values,
                       bool send_index,
                       int def)
: FormElement(parent, name, required), values_(values), send_index_(send_index), def_(def)
{
  auto layout = new QVBoxLayout(this);
  combo_ = new QComboBox(this);
  combo_->installEventFilter(this);
  combo_->setFocusPolicy(Qt::StrongFocus);
  layout->addWidget(combo_);
  for(const auto & v : values) { combo_->addItem(v.c_str()); }
  reset();
}

void ComboInput::reset()
{
  combo_->disconnect();
  combo_->setCurrentIndex(def_);
  ready_ = (def_ >= 0);
  connect(combo_, SIGNAL(currentIndexChanged(int)), this, SLOT(currentIndexChanged(int)));
}

void ComboInput::changed(bool required, const std::vector<std::string> & values, bool send_index, int def)
{
  if(combo_->hasFocus()) { return; }
  changed_(required);
  def_ = def;
  values_ = values;
  send_index_ = send_index;
  reset();
}

void ComboInput::currentIndexChanged(int idx)
{
  ready_ = idx != -1;
  locked_ = true;
}

mc_rtc::Configuration ComboInput::serialize() const
{
  if(send_index_) { return details::serialize(combo_->currentIndex()); }
  else { return details::serialize(combo_->currentText().toStdString()); }
}

DataComboInput::DataComboInput(QWidget * parent,
                               const std::string & name,
                               bool required,
                               const mc_rtc::Configuration & dataIn,
                               const std::vector<std::string> & ref,
                               bool send_index,
                               const std::string & rename)
: FormElement(parent, name, required), send_index_(send_index), data_(dataIn), ref_(ref), resolved_ref_(ref),
  rename_(rename.size() ? rename : name)
{
  auto layout = new QVBoxLayout(this);
  combo_ = new QComboBox(this);
  combo_->installEventFilter(this);
  combo_->setFocusPolicy(Qt::StrongFocus);
  layout->addWidget(combo_);
  reset();
}

void DataComboInput::reset()
{
  combo_->disconnect();
  combo_->clear();
  values_.clear();
  resolved_ref_ = ref_;
  if(resolve_ref()) { update_values(); }
  combo_->setCurrentIndex(-1);
  ready_ = false;
  connect(combo_, SIGNAL(currentIndexChanged(int)), this, SLOT(currentIndexChanged(int)));
}

void DataComboInput::changed(bool required,
                             const mc_rtc::Configuration &,
                             const std::vector<std::string> & ref,
                             bool send_index)
{
  if(combo_->hasFocus()) { return; }
  changed_(required);
  ref_ = ref;
  send_index_ = send_index;
  if(resolve_ref()) { update_values(); }
}

void DataComboInput::currentIndexChanged(int idx)
{
  ready_ = idx != -1;
  locked_ = true;
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
        if(combo->currentIndex() != -1) { update_field(other->name(), combo->currentText().toStdString()); }
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
  if(send_index_) { return details::serialize(combo_->currentIndex()); }
  else { return details::serialize(combo_->currentText().toStdString()); }
}

bool DataComboInput::resolve_ref()
{
  for(const auto & k : resolved_ref_)
  {
    if(k.size() == 0 || k[0] == '$') { return false; }
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
      if(k == name) { resolved_ref_[i] = value; }
    }
  }
  if(resolve_ref()) { update_values(); }
}

void DataComboInput::update_values()
{
  auto dataIn = data_;
  for(const auto & k : resolved_ref_) { dataIn = dataIn(k, mc_rtc::Configuration{}); }
  auto values = dataIn.size() ? dataIn : std::vector<std::string>{};
  std::sort(values.begin(), values.end(),
            [](const std::string & lhs, const std::string & rhs)
            { return QString::compare(lhs.c_str(), rhs.c_str(), Qt::CaseInsensitive) < 0; });
  if(values_ != values)
  {
    auto selected = combo_->currentText().toStdString();
    int idx = -1;
    values_ = values;
    combo_->clear();
    for(size_t i = 0; i < values.size(); ++i)
    {
      const auto & v = values[i];
      if(v == selected) { idx = static_cast<int>(i); }
      combo_->addItem(v.c_str());
    }
    if(idx != -1) { combo_->setCurrentIndex(idx); }
    else if(values.size()) { combo_->setCurrentIndex(0); }
  }
}

Form::Form(QWidget * parent,
           const std::string & name,
           bool required,
           const std::vector<FormElement *> elements,
           bool checkable,
           bool use_group_name,
           bool tuple)
: FormElement(parent, name, required), elements_(elements), tuple_(tuple)
{
  spanning_ = true;
  show_name_ = false;
  auto title = required ? name + "*" : name;
  auto m_layout = new QVBoxLayout(this);
  group_ = new QGroupBox(use_group_name ? title.c_str() : "", parent);
  group_->setStyleSheet("QGroupBox {border: 1px solid gray;border-radius: 9px;margin-top: 0.5em;} QGroupBox::title { "
                        "subcontrol-origin: margin; left: 10px; margin-bottom: 2em; padding: 0 3px 0 3px;}");
  if(tuple_)
  {
    auto layout = new QHBoxLayout(group_);
    for(auto & el : elements_)
    {
      for(auto & other : elements_) { el->update_dependencies(other); }
      layout->addWidget(el);
    }
  }
  else
  {
    auto layout = new QFormLayout(group_);
    for(auto & el : elements_)
    {
      for(auto & other : elements_) { el->update_dependencies(other); }
      auto el_name = el->required() ? el->name() + "*" : el->name();
      if(!el->spanning()) { layout->addRow(el_name.c_str(), el); }
      else
      {
        if(el->show_name()) { layout->addRow(new QLabel(el_name.c_str(), this)); }
        layout->addRow(el);
      }
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
  bool ok = true;
  for(const auto & el : elements_)
  {
    if(el->required() && !el->ready()) { return false; }
  }
  return ok;
}

mc_rtc::Configuration Form::serialize() const
{
  return serialize(tuple_);
}

mc_rtc::Configuration Form::serialize(bool asTuple) const
{
  mc_rtc::Configuration out;
  if(asTuple) { out = out.array("data", elements_.size()); }
  for(auto & el : elements_)
  {
    if(el->ready())
    {
      if(asTuple) { out.push(el->serialize()); }
      else { out.add(el->name(), el->serialize()); }
    }
  }
  return out;
}

void Form::rejectUncheck()
{
  group_->setChecked(true);
}

void Form::reset()
{
  for(auto & el : elements_) { el->reset(); }
}

std::string formId2name(const WidgetId & id, const std::string & name)
{
  std::string ret;
  for(const auto c : id.category) { ret += c + "/"; }
  ret += id.name;
  ret += "/";
  ret += name;
  return ret;
}

namespace details
{

template<typename DataT, bool rotation_only>
InteractiveMarkerInput<DataT, rotation_only>::InteractiveMarkerInput(
    QWidget * parent,
    const std::string & name,
    bool required,
    const WidgetId & formId,
    const DataT & default_,
    bool default_from_user,
    bool interactive,
    std::shared_ptr<interactive_markers::InteractiveMarkerServer> int_server)
: FormElement(parent, name, required),
  marker_(int_server,
          formId2name(formId, name),
          make6DMarker(formId2name(formId, name),
                       makeAxisMarker(0.15 * 0.9),
                       (!rotation_only) && interactive,
                       std::is_same_v<DataT, sva::PTransformd> && interactive),

          [this](const visualization_msgs::InteractiveMarkerFeedbackConstPtr & feedback) { handleRequest(feedback); }),
  data_(default_)
{
  auto layout = new QGridLayout(this);
  auto validator = new QDoubleValidator(this);
  validator->setLocale(QLocale::C);
  int row = 0;
  if constexpr(!rotation_only)
  {
    auto & data = [this]() -> Eigen::Vector3d &
    {
      if constexpr(std::is_same_v<Eigen::Vector3d, DataT>) { return data_; }
      else { return data_.translation(); }
    }();
    int column = 0;
    for(const auto & l : {"x", "y", "z"})
    {
      layout->addWidget(new QLabel(l), 0, column);
      auto edit = new QLineEdit(QString::number(data(column)), this);
      edit->setValidator(validator);
      edits_[static_cast<size_t>(column)] = edit;
      layout->addWidget(edit, 1, column);
      this->connect(edit, &QLineEdit::textChanged, this,
                    [this, column, &data](const QString & txt)
                    {
                      ready_ = true;
                      locked_ = true;
                      data(column) = txt.toDouble();
                      marker_.update(data_);
                    });
      column++;
    }
    row++;
  }
  if constexpr(std::is_same_v<DataT, sva::PTransformd>)
  {
    auto get_data = [this]() { return Eigen::Quaterniond(data_.rotation()); };
    auto data = get_data();
    auto get_value = [](Eigen::Quaterniond & data, const char label) -> double &
    {
      if(label == 'w') { return data.w(); }
      if(label == 'x') { return data.x(); }
      if(label == 'y') { return data.y(); }
      if(label == 'z') { return data.z(); }
      mc_rtc::log::error_and_throw("unreacheable");
    };
    int column = 0;
    for(const auto & l : {'w', 'x', 'y', 'z'})
    {
      layout->addWidget(new QLabel(QString(l)), 2 * row, column);
      auto edit = new QLineEdit(QString::number(get_value(data, l)), this);
      edit->setValidator(validator);
      edits_[static_cast<size_t>(3 * row + column)] = edit;
      layout->addWidget(edit, 2 * row + 1, column);
      column++;
      this->connect(edit, &QLineEdit::textChanged, this,
                    [this, l, get_data, get_value](const QString & txt)
                    {
                      ready_ = true;
                      locked_ = true;
                      auto data = get_data();
                      get_value(data, l) = txt.toDouble();
                      data_.rotation() = Eigen::Matrix3d(data.normalized());
                      marker_.update(data_);
                    });
    }
  }
  changed(required, formId, default_, default_from_user, interactive, int_server);
}

template<typename DataT, bool rotation_only>
void InteractiveMarkerInput<DataT, rotation_only>::changed(
    bool required,
    const WidgetId & formId,
    const DataT & default_,
    bool default_from_user,
    bool interactive,
    std::shared_ptr<interactive_markers::InteractiveMarkerServer> int_server)
{
  if(std::any_of(edits_.begin(), edits_.end(), [](const auto * edit) { return edit->hasFocus(); })) { return; }
  changed_(required);
  data_ = default_;
  default_data_ = data_;
  user_default_ = default_from_user;
  reset();
}

template<typename DataT, bool rotation_only>
mc_rtc::Configuration InteractiveMarkerInput<DataT, rotation_only>::serialize() const
{
  mc_rtc::Configuration out;
  if constexpr(std::is_same_v<DataT, sva::PTransformd> && rotation_only) { out.add("data", data_.rotation()); }
  else { out.add("data", data_); }
  return out("data");
}

template<typename DataT, bool rotation_only>
void InteractiveMarkerInput<DataT, rotation_only>::reset()
{
  data_ = default_data_;
  ready_ = user_default_;
  reset_edits();
  marker_.update(data_);
}

template<typename DataT, bool rotation_only>
void InteractiveMarkerInput<DataT, rotation_only>::handleRequest(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr & feedback)
{
  locked_ = true;
  ready_ = true;
  if constexpr(std::is_same_v<DataT, Eigen::Vector3d>)
  {
    data_.x() = feedback->pose.position.x;
    data_.y() = feedback->pose.position.y;
    data_.z() = feedback->pose.position.z;
  }
  else
  {
    if constexpr(!rotation_only)
    {
      data_.translation().x() = feedback->pose.position.x;
      data_.translation().y() = feedback->pose.position.y;
      data_.translation().z() = feedback->pose.position.z;
    }
    const auto & ori = feedback->pose.orientation;
    Eigen::Quaterniond q(ori.w, ori.x, ori.y, ori.z);
    data_.rotation() = q.normalized().toRotationMatrix().transpose();
  }
  reset_edits();
}

template<typename DataT, bool rotation_only>
void InteractiveMarkerInput<DataT, rotation_only>::reset_edits()
{
  size_t idx = 0;
  auto setText = [](QLineEdit * edit, double number)
  {
    edit->blockSignals(true);
    edit->setText(QString::number(number));
    edit->blockSignals(false);
  };
  if constexpr(!rotation_only)
  {
    auto & data = [this]() -> Eigen::Vector3d &
    {
      if constexpr(std::is_same_v<Eigen::Vector3d, DataT>) { return data_; }
      else { return data_.translation(); }
    }();
    setText(edits_[0], data(0));
    setText(edits_[1], data(1));
    setText(edits_[2], data(2));
    idx = 3;
  }
  if constexpr(std::is_same_v<DataT, sva::PTransformd>)
  {
    auto data = Eigen::Quaterniond(data_.rotation());
    setText(edits_[idx], data.w());
    setText(edits_[idx + 1], data.x());
    setText(edits_[idx + 2], data.y());
    setText(edits_[idx + 3], data.z());
  }
}

template struct InteractiveMarkerInput<Eigen::Vector3d, false>;
template struct InteractiveMarkerInput<sva::PTransformd, false>;
template struct InteractiveMarkerInput<sva::PTransformd, true>;

} // namespace details

Object::Object(QWidget * parent, const std::string & name, bool required, FormElementContainer * parentForm)
: FormElement(parent, name, required)
{
  auto layout = new QVBoxLayout(this);
  container_ = new FormElementContainer(this, parentForm);
  layout->addWidget(container_);
}

void Object::changed(bool required, FormElementContainer *)
{
  changed_(required);
}

mc_rtc::Configuration Object::serialize() const
{
  mc_rtc::Configuration out;
  container_->collect(out);
  return out;
}

void Object::reset()
{
  container_->reset();
}

bool Object::ready() const
{
  return container_->ready();
}

bool Object::locked() const
{
  return container_->locked();
}

void Object::unlock()
{
  container_->unlock();
}

} // namespace form

} // namespace mc_rtc_rviz
