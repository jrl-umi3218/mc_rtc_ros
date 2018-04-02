#pragma once

#include <QtWidgets>

#include <mc_rtc/Configuration.h>

namespace mc_rtc_rviz
{

namespace form
{
  struct Form;
}

struct FormElement : public QWidget
{
  friend struct form::Form;

  FormElement(QWidget * parent, const std::string & name, bool required);

  virtual ~FormElement() = default;

  bool eventFilter( QObject * o, QEvent * e ) override;

  bool fill(mc_rtc::Configuration & out, std::string & msg);

  virtual bool ready() const { return ready_; }

  bool required() const { return required_; }

  bool show_name() const { return show_name_; }

  bool spanning() const { return spanning_; }

  const std::string & name() const { return name_; }

  void name(const std::string & name) { name_ = name; }

  void update_dependencies(const std::vector<FormElement*> & others);

  virtual void update_dependencies(FormElement * other) {}
protected:
  virtual void fill_(mc_rtc::Configuration & out) = 0;

  bool ready_ = false;
  bool spanning_ = false;
  bool show_name_ = true;
private:
  std::string name_;
  bool required_;
};

namespace form
{

struct Checkbox : public FormElement
{
  Checkbox(QWidget * parent, const std::string & name, bool required, bool def);
protected:
  void fill_(mc_rtc::Configuration & out) override;
private:
  QCheckBox * cbox_;
};

struct IntegerInput : public FormElement
{
  IntegerInput(QWidget * parent, const std::string & name, bool required, int def);
protected:
  void fill_(mc_rtc::Configuration & out) override;
private:
  QLineEdit * edit_;
};

struct NumberInput : public FormElement
{
  NumberInput(QWidget * parent, const std::string & name, bool required, double def);
protected:
  void fill_(mc_rtc::Configuration & out) override;
private:
  QLineEdit * edit_;
};

struct StringInput : public FormElement
{
  StringInput(QWidget * parent, const std::string & name, bool required, const std::string & def);
protected:
  void fill_(mc_rtc::Configuration & out) override;
private:
  QLineEdit * edit_;
};

template<typename T>
struct ArrayInput : public FormElement
{
  ArrayInput(QWidget * parent, const std::string & name, bool required, bool fixed_size, int min_size = 0, int max_size = 256);
protected:
  void fill_(mc_rtc::Configuration & out) override;
  void add_edit(const T & def);
private:
  void add_validator(QLineEdit * edit) {}
  void data2edit(const T & value, QLineEdit * edit);
  T edit2data(QLineEdit * edit);
  bool fixed_size_;
  int min_size_;
  int max_size_;
  QGridLayout * layout_;
  QPushButton * add_button_ = nullptr;
  std::vector<QLineEdit*> edits_;
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
void ArrayInput<std::string>::data2edit(const std::string & value, QLineEdit * edit);
template<>
std::string ArrayInput<std::string>::edit2data(QLineEdit * edit);

template<typename T>
ArrayInput<T>::ArrayInput(QWidget * parent, const std::string & name, bool required, bool fixed_size, int min_size, int max_size)
: FormElement(parent, name, required),
  fixed_size_(fixed_size), min_size_(min_size), max_size_(max_size)
{
  spanning_ = true;
  auto mainLayout = new QVBoxLayout(this);
  auto w = new QWidget(this);
  mainLayout->QLayout::addWidget(w);
  layout_ = new QGridLayout(w);
  for(int i = 0; i < min_size; ++i)
  {
    add_edit(T{});
  }
  if(!fixed_size)
  {
    add_button_ = new QPushButton("+");
    connect(add_button_, &QPushButton::released,
            this, [this](){ ready_ = true; add_edit(T{}); });
    mainLayout->addWidget(add_button_);
    if(edits_.size() == max_size_) { add_button_->hide(); }
  }
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
  connect(edit, &QLineEdit::textChanged,
          this, [this](const QString&) { ready_ = true; });
  auto row = layout_->rowCount();
  layout_->addWidget(edit, row, 0);
  edits_.push_back(edit);
  if(edits_.size() == max_size_ && add_button_)
  {
    add_button_->hide();
  }
  if(!fixed_size_ && edits_.size() > min_size_)
  {
    auto button = new QPushButton("-");
    connect(button, &QPushButton::released,
            this, [this,edit,button]()
            {
              ready_ = true;
              edits_.erase(std::find(edits_.begin(), edits_.end(), edit));
              if(edits_.size() < max_size_ && add_button_)
              {
                add_button_->show();
              }
              delete edit;
              delete button;
            });
    layout_->addWidget(button, row, 1);
  }
}

using IntegerArrayInput = ArrayInput<int>;
using StringArrayInput = ArrayInput<std::string>;

struct NumberArrayInput : public ArrayInput<double>
{
  NumberArrayInput(QWidget * parent, const std::string & name, bool required, const Eigen::VectorXd & def, bool fixed_size);
  NumberArrayInput(QWidget * parent, const std::string & name, bool required, bool fixed_size, int min_size, int max_size);
};


struct ComboInput : public FormElement
{
  ComboInput(QWidget * parent, const std::string & name, bool required, const std::vector<std::string> & values, bool send_index);
protected:
  void fill_(mc_rtc::Configuration & out) override;
private:
  bool send_index_;
  QComboBox * combo_;
};

struct DataComboInput : public FormElement
{
  DataComboInput(QWidget * parent, const std::string & name, bool required, const mc_rtc::Configuration & data, const std::vector<std::string> & ref, bool send_index, const std::string & rename = "");

  void update_dependencies(FormElement * other) override;
protected:
  void fill_(mc_rtc::Configuration & out) override;
private:
  bool send_index_;
  std::string rename_;
  const mc_rtc::Configuration & data_;
  std::vector<std::string> ref_;
  std::vector<std::string> resolved_ref_;

  QComboBox * combo_;

  std::set<std::string> connected_;

  bool resolve_ref();

  void update_field(const std::string & name, const std::string & value);

  void update_values();
};

struct Form : public FormElement
{
  Form(QWidget * parent, const std::string & name, bool required, const std::vector<FormElement*> elements);

  bool ready() const override;
protected:
  void fill_(mc_rtc::Configuration & out) override;
private:
  std::vector<FormElement*> elements_;
};

}

}
