/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <QtGlobal>
#if QT_VERSION < QT_VERSION_CHECK(5, 0, 0)
#  include <QtGui>
#else
#  include <QtWidgets>
#endif

#include <mc_rtc/Configuration.h>

namespace mc_rtc_rviz
{

namespace form
{
struct Form;
}

struct FormElement : public QWidget
{
  Q_OBJECT
public:
  friend struct form::Form;

  FormElement(QWidget * parent, const std::string & name, bool required);

  virtual ~FormElement() = default;

  bool eventFilter(QObject * o, QEvent * e) override;

  bool fill(mc_rtc::Configuration & out, std::string & msg);

  virtual bool ready() const
  {
    return ready_;
  }

  bool required() const
  {
    return required_;
  }

  bool show_name() const
  {
    return show_name_;
  }

  bool spanning() const
  {
    return spanning_;
  }

  bool hidden() const
  {
    return hidden_;
  }

  void hidden(bool h)
  {
    hidden_ = h;
  }

  const std::string & name() const
  {
    return name_;
  }

  void name(const std::string & name)
  {
    name_ = name;
  }

  size_t type()
  {
    return type_;
  }

  void type(size_t t)
  {
    type_ = t;
  }

  virtual void update_dependencies(FormElement *) {}

protected:
  virtual void fill_(mc_rtc::Configuration & out) = 0;

  bool changed_(bool required)
  {
    return required_ != required;
  }

  bool ready_ = false;
  bool spanning_ = false;
  bool show_name_ = true;
  bool hidden_ = false;

  bool seen_ = true;
  bool required_;

private:
  std::string name_;
  size_t type_;
};

namespace form
{

struct Checkbox : public FormElement
{
  Q_OBJECT
public:
  Checkbox(QWidget * parent, const std::string & name, bool required, bool def);

  bool changed(bool required, bool def);

protected:
  void fill_(mc_rtc::Configuration & out) override;

private:
  QCheckBox * cbox_;
  bool def_;
private slots:
  void toggled(bool);
};

struct CommonInput : public FormElement
{
  Q_OBJECT
protected:
  CommonInput(QWidget * parent, const std::string & name, bool required);

  QLineEdit * edit_;
private slots:
  void textChanged(const QString &);
};

struct IntegerInput : public CommonInput
{
  IntegerInput(QWidget * parent, const std::string & name, bool required, int def);

  bool changed(bool required, int def);

protected:
  void fill_(mc_rtc::Configuration & out) override;

private:
  int def_;
};

struct NumberInput : public CommonInput
{
  NumberInput(QWidget * parent, const std::string & name, bool required, double def);

  bool changed(bool required, double def);

protected:
  void fill_(mc_rtc::Configuration & out) override;

private:
  double def_;
};

struct StringInput : public CommonInput
{
  StringInput(QWidget * parent, const std::string & name, bool required, const std::string & def);

  bool changed(bool required, const std::string & def);

protected:
  void fill_(mc_rtc::Configuration & out) override;

private:
  std::string def_;
};

struct CommonArrayInput : public FormElement
{
  Q_OBJECT
protected:
  CommonArrayInput(QWidget * parent, const std::string & name, bool required);
protected slots:
  virtual void plusReleased() = 0;
  virtual void minusReleased() = 0;
  void textChanged(const QString &);
};

template<typename T>
struct ArrayInput : public CommonArrayInput
{
  ArrayInput(QWidget * parent,
             const std::string & name,
             bool required,
             bool fixed_size,
             int min_size = 0,
             int max_size = 256);

protected:
  void fill_(mc_rtc::Configuration & out) override;
  void add_edit(const T & def);

private:
  void add_validator(QLineEdit * edit);
  void data2edit(const T & value, QLineEdit * edit);
  T edit2data(QLineEdit * edit);

protected:
  bool fixed_size_;

private:
  int min_size_;
  int max_size_;
  int stride_ = 1;
  int next_row_ = 0;
  int next_column_ = 0;
  QGridLayout * layout_;
  QPushButton * add_button_ = nullptr;
  std::vector<QLineEdit *> edits_;

protected: // Implement virtual slots
  void plusReleased() override;
  void minusReleased() override;
};

template<>
void ArrayInput<int>::add_validator(QLineEdit * edit);
template<>
void ArrayInput<double>::add_validator(QLineEdit * edit);
template<>
void ArrayInput<int>::data2edit(const int & value, QLineEdit * edit);
template<>
int ArrayInput<int>::edit2data(QLineEdit * edit);
template<>
void ArrayInput<double>::data2edit(const double & value, QLineEdit * edit);
template<>
double ArrayInput<double>::edit2data(QLineEdit * edit);
template<>
void ArrayInput<std::string>::add_validator(QLineEdit *);
template<>
void ArrayInput<std::string>::data2edit(const std::string & value, QLineEdit * edit);
template<>
std::string ArrayInput<std::string>::edit2data(QLineEdit * edit);

template<typename T>
ArrayInput<T>::ArrayInput(QWidget * parent,
                          const std::string & name,
                          bool required,
                          bool fixed_size,
                          int min_size,
                          int max_size)
: CommonArrayInput(parent, name, required), fixed_size_(fixed_size), min_size_(min_size), max_size_(max_size)
{
  if(min_size == max_size && (min_size == 9 || min_size == 36))
  {
    stride_ = min_size == 9 ? 3 : 6;
  }
  spanning_ = true;
  auto mainLayout = new QVBoxLayout(this);
  auto w = new QWidget(this);
  mainLayout->QLayout::addWidget(w);
  layout_ = new QGridLayout(w);
  for(int i = 0; i < min_size; ++i)
  {
    if(stride_ == 1)
    {
      add_edit(T{});
    }
    else
    {
      add_edit(i % stride_ == next_row_ ? T{1} : T{0});
    }
  }
  if(!fixed_size)
  {
    add_button_ = new QPushButton("+");
    connect(add_button_, SIGNAL(released()), this, SLOT(plusReleased()));
    mainLayout->addWidget(add_button_);
    if(edits_.size() == max_size_)
    {
      add_button_->hide();
    }
  }
}

template<typename T>
void ArrayInput<T>::plusReleased()
{
  ready_ = true;
  add_edit(T{});
}

template<typename T>
void ArrayInput<T>::fill_(mc_rtc::Configuration & out)
{
  std::vector<T> data;
  data.reserve(edits_.size());
  for(auto e : edits_)
  {
    data.push_back(edit2data(e));
  }
  out.add(name(), data);
}

template<typename T>
void ArrayInput<T>::add_edit(const T & def)
{
  auto edit = new QLineEdit(this);
  add_validator(edit);
  data2edit(def, edit);
  connect(edit, SIGNAL(textChanged(const QString &)), this, SLOT(textChanged(const QString &)));
  layout_->addWidget(edit, next_row_, next_column_++);
  edits_.push_back(edit);
  if(edits_.size() == max_size_ && add_button_)
  {
    add_button_->hide();
  }
  if(!fixed_size_ && edits_.size() > min_size_)
  {
    auto button = new QPushButton("-");
    connect(button, SIGNAL(released()), this, SLOT(minusReleased()));
    layout_->addWidget(button, next_row_, 1);
  }
  if(next_column_ == stride_)
  {
    next_column_ = 0;
    next_row_++;
  }
}

template<typename T>
void ArrayInput<T>::minusReleased()
{
  ready_ = true;
  auto button = qobject_cast<QWidget *>(sender());
  QLineEdit * edit = nullptr;
  for(int i = 0; i < layout_->rowCount(); ++i)
  {
    auto item = layout_->itemAtPosition(i, 1);
    if(!item)
    {
      continue;
    }
    auto w = item->widget();
    if(w == button)
    {
      edit = qobject_cast<QLineEdit *>(layout_->itemAtPosition(i, 0)->widget());
      break;
    }
  }
  edits_.erase(std::find(edits_.begin(), edits_.end(), edit));
  if(edits_.size() < max_size_ && add_button_)
  {
    add_button_->show();
  }
  delete edit;
  delete button;
}

using IntegerArrayInput = ArrayInput<int>;
using StringArrayInput = ArrayInput<std::string>;

struct NumberArrayInput : public ArrayInput<double>
{
  NumberArrayInput(QWidget * parent,
                   const std::string & name,
                   bool required,
                   const Eigen::VectorXd & def,
                   bool fixed_size);
  NumberArrayInput(QWidget * parent,
                   const std::string & name,
                   bool required,
                   bool fixed_size,
                   int min_size,
                   int max_size);

  bool changed(bool required, const Eigen::VectorXd & def, bool fixed_size);

private:
  Eigen::VectorXd def_;
};

struct ComboInput : public FormElement
{
  Q_OBJECT
public:
  ComboInput(QWidget * parent,
             const std::string & name,
             bool required,
             const std::vector<std::string> & values,
             bool send_index);

  bool changed(bool required, const std::vector<std::string> & values, bool send_index);

protected:
  void fill_(mc_rtc::Configuration & out) override;

private:
  std::vector<std::string> values_;
  bool send_index_;
  QComboBox * combo_;
private slots:
  void currentIndexChanged(int);
};

struct DataComboInput : public FormElement
{
  Q_OBJECT
public:
  DataComboInput(QWidget * parent,
                 const std::string & name,
                 bool required,
                 const mc_rtc::Configuration & data,
                 const std::vector<std::string> & ref,
                 bool send_index,
                 const std::string & rename = "");

  bool changed(bool required,
               const mc_rtc::Configuration & data,
               const std::vector<std::string> & ref,
               bool send_index);

  void update_dependencies(FormElement * other) override;

protected:
  void fill_(mc_rtc::Configuration & out) override;

private:
  bool send_index_;
  const mc_rtc::Configuration & data_;
  std::vector<std::string> ref_;
  std::vector<std::string> resolved_ref_;
  std::vector<std::string> values_;
  std::string rename_;

  QComboBox * combo_;

  std::set<std::string> connected_;

  bool resolve_ref();

  void update_field(const std::string & name, const std::string & value);

  void update_values();
private slots:
  void currentIndexChanged(int);
  void currentIndexChanged(const QString &);
};

struct Form : public FormElement
{
  Form(QWidget * parent, const std::string & name, bool required, const std::vector<FormElement *> elements);

  bool ready() const override;

protected:
  void fill_(mc_rtc::Configuration & out) override;

private:
  std::vector<FormElement *> elements_;
};

} // namespace form

} // namespace mc_rtc_rviz
