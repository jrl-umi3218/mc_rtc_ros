/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include "Qt.h"

#include <mc_rtc/Configuration.h>

#include "ClientWidget.h"
#include "utils.h"

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

  bool can_fill(std::string & msg);

  virtual mc_rtc::Configuration serialize() const = 0;

  virtual void reset() = 0;

  virtual bool ready() const { return ready_; }

  inline bool locked() const { return locked_; }

  inline void unlock() { locked_ = false; }

  bool required() const { return required_; }

  bool show_name() const { return show_name_; }

  bool spanning() const { return spanning_; }

  bool hidden() const { return hidden_; }

  void hidden(bool h) { hidden_ = h; }

  const std::string & name() const { return name_; }

  void name(const std::string & name) { name_ = name; }

  size_t type() { return type_; }

  void type(size_t t) { type_ = t; }

  virtual void update_dependencies(FormElement *) {}

public slots:
  inline void unlocked() { locked_ = false; }

protected:
  void changed_(bool required) { required_ = required; }

  bool ready_ = false;
  bool spanning_ = false;
  bool show_name_ = true;
  bool hidden_ = false;
  bool locked_ = false;

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
  Checkbox(QWidget * parent, const std::string & name, bool required, bool def, bool user_def);

  void changed(bool required, bool def, bool user_def);

  mc_rtc::Configuration serialize() const override;

  void reset() override;

private:
  QCheckBox * cbox_;
  bool def_;
  bool user_def_;
private slots:
  void toggled(bool);
  void stateChanged(int);
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
  IntegerInput(QWidget * parent, const std::string & name, bool required, int def, bool user_def);

  void changed(bool required, int def, bool user_def);

  mc_rtc::Configuration serialize() const override;

  void reset() override;

private:
  int def_;
  bool user_def_;
};

struct NumberInput : public CommonInput
{
  NumberInput(QWidget * parent, const std::string & name, bool required, double def, bool user_def);

  void changed(bool required, double def, bool user_def);

  mc_rtc::Configuration serialize() const override;

  void reset() override;

private:
  double def_;
  bool user_def_;
};

struct StringInput : public CommonInput
{
  StringInput(QWidget * parent, const std::string & name, bool required, const std::string & def, bool user_def);

  void changed(bool required, const std::string & def, bool user_def);

  mc_rtc::Configuration serialize() const override;

  void reset() override;

private:
  std::string def_;
  bool user_def_;
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

  mc_rtc::Configuration serialize() const override;

  void reset() override;

protected:
  void add_edit(const T & def);
  void data2edit(const T & value, QLineEdit * edit);

private:
  void add_validator(QLineEdit * edit);
  T edit2data(QLineEdit * edit) const;

protected:
  bool fixed_size_;

private:
  int min_size_;
  int max_size_;
  bool is_square_ = false;
  int stride_ = 1;
  int next_row_ = 0;
  int next_column_ = 0;
  QGridLayout * layout_;
  QPushButton * add_button_ = nullptr;

protected: // Implement virtual slots
  void plusReleased() override;
  void minusReleased() override;
  void updateStride(size_t size);
  std::vector<QLineEdit *> edits_;
};

template<>
void ArrayInput<int>::add_validator(QLineEdit * edit);
template<>
void ArrayInput<double>::add_validator(QLineEdit * edit);
template<>
void ArrayInput<int>::data2edit(const int & value, QLineEdit * edit);
template<>
int ArrayInput<int>::edit2data(QLineEdit * edit) const;
template<>
void ArrayInput<double>::data2edit(const double & value, QLineEdit * edit);
template<>
double ArrayInput<double>::edit2data(QLineEdit * edit) const;
template<>
void ArrayInput<std::string>::add_validator(QLineEdit *);
template<>
void ArrayInput<std::string>::data2edit(const std::string & value, QLineEdit * edit);
template<>
std::string ArrayInput<std::string>::edit2data(QLineEdit * edit) const;

template<typename T>
ArrayInput<T>::ArrayInput(QWidget * parent,
                          const std::string & name,
                          bool required,
                          bool fixed_size,
                          int min_size,
                          int max_size)
: CommonArrayInput(parent, name, required), fixed_size_(fixed_size), min_size_(min_size), max_size_(max_size)
{
  if(min_size == max_size) { updateStride(min_size); }
  spanning_ = true;
  auto mainLayout = new QVBoxLayout(this);
  auto w = new QWidget(this);
  mainLayout->QLayout::addWidget(w);
  layout_ = new QGridLayout(w);
  reset();
  if(!fixed_size)
  {
    add_button_ = new QPushButton("+");
    connect(add_button_, SIGNAL(released()), this, SLOT(plusReleased()));
    mainLayout->addWidget(add_button_);
    if(edits_.size() == static_cast<size_t>(max_size_)) { add_button_->hide(); }
  }
}

template<typename T>
void ArrayInput<T>::reset()
{
  for(int i = 0; i < layout_->rowCount(); ++i)
  {
    for(int j = 0; j < layout_->columnCount(); ++j)
    {
      auto itm = layout_->itemAtPosition(i, j);
      if(itm) { itm->widget()->deleteLater(); }
    }
  }
  edits_.clear();
  for(int i = 0; i < min_size_; ++i)
  {
    if(is_square_) { add_edit(i % stride_ == next_row_ ? T{1} : T{0}); }
    else { add_edit(T{}); }
  }
}

template<typename T>
void ArrayInput<T>::updateStride(size_t size)
{
  if(size < 6) { stride_ = static_cast<int>(size); }
  else if(size == 9)
  {
    is_square_ = true;
    stride_ = 3;
  }
  else if(size == 16)
  {
    is_square_ = true;
    stride_ = 4;
  }
  else if(size == 36)
  {
    is_square_ = true;
    stride_ = 6;
  }
  else { stride_ = 6; }
}

template<typename T>
void ArrayInput<T>::plusReleased()
{
  ready_ = true;
  add_edit(T{});
}

template<typename T>
mc_rtc::Configuration ArrayInput<T>::serialize() const
{
  mc_rtc::Configuration c;
  auto out = c.array("DATA", edits_.size());
  for(auto e : edits_) { out.push(edit2data(e)); }
  return out;
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
  if(edits_.size() == static_cast<size_t>(max_size_) && add_button_) { add_button_->hide(); }
  if(!fixed_size_ && static_cast<int>(edits_.size()) > min_size_)
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
    if(!item) { continue; }
    auto w = item->widget();
    if(w == button)
    {
      edit = qobject_cast<QLineEdit *>(layout_->itemAtPosition(i, 0)->widget());
      break;
    }
  }
  edits_.erase(std::find(edits_.begin(), edits_.end(), edit));
  if(edits_.size() < static_cast<size_t>(max_size_) && add_button_) { add_button_->show(); }
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
                   bool fixed_size,
                   bool user_def);
  NumberArrayInput(QWidget * parent,
                   const std::string & name,
                   bool required,
                   bool fixed_size,
                   int min_size,
                   int max_size);

  void changed(bool required, const Eigen::VectorXd & def, bool fixed_size, bool user_def);

  void reset() override;

private:
  Eigen::VectorXd def_;
  bool user_def_;
};

struct ComboInput : public FormElement
{
  Q_OBJECT
public:
  ComboInput(QWidget * parent,
             const std::string & name,
             bool required,
             const std::vector<std::string> & values,
             bool send_index,
             int def);

  void changed(bool required, const std::vector<std::string> & values, bool send_index, int def);

  mc_rtc::Configuration serialize() const override;

  void reset() override;

private:
  std::vector<std::string> values_;
  bool send_index_;
  int def_;
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

  void changed(bool required,
               const mc_rtc::Configuration & data,
               const std::vector<std::string> & ref,
               bool send_index);

  void update_dependencies(FormElement * other) override;

  mc_rtc::Configuration serialize() const override;

  void reset() override;

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
  Q_OBJECT
public:
  Form(QWidget * parent,
       const std::string & name,
       bool required,
       const std::vector<FormElement *> elements,
       bool checkable = false,
       bool use_group_name = true,
       bool tuple = false);

  bool ready() const override;

  void rejectUncheck();

  mc_rtc::Configuration serialize() const override;

  mc_rtc::Configuration serialize(bool asTuple) const;

  void reset() override;

private:
  std::vector<FormElement *> elements_;
  QGroupBox * group_;
  bool tuple_;

signals:
  /** Emitted if the Form is checkable and the users unchecks it */
  void toggled(bool);
};

namespace details
{

template<typename DataT, bool rotation_only>
struct InteractiveMarkerInput : public FormElement
{
public:
  InteractiveMarkerInput(QWidget * parent,
                         const std::string & name,
                         bool required,
                         const WidgetId & formId,
                         const DataT & default_,
                         bool default_from_user,
                         bool interactive,
                         std::shared_ptr<interactive_markers::InteractiveMarkerServer> int_server);

  void changed(bool required,
               const WidgetId & formId,
               const DataT & default_,
               bool default_from_user,
               bool interactive,
               std::shared_ptr<interactive_markers::InteractiveMarkerServer> int_server);

  mc_rtc::Configuration serialize() const override;

  void reset() override;

private:
  SharedMarker marker_;
  DataT data_;
  bool user_default_;

  void handleRequest(const visualization_msgs::InteractiveMarkerFeedbackConstPtr & feedback);

  static constexpr size_t n_edits = std::is_same_v<Eigen::Vector3d, DataT> ? 3 : (rotation_only ? 4 : 7);
  std::array<QLineEdit *, n_edits> edits_;

  void reset_edits();
};

extern template struct InteractiveMarkerInput<Eigen::Vector3d, false>;
extern template struct InteractiveMarkerInput<sva::PTransformd, false>;
extern template struct InteractiveMarkerInput<sva::PTransformd, true>;

} // namespace details

struct Point3DInput : public details::InteractiveMarkerInput<Eigen::Vector3d, false>
{
  Q_OBJECT
public:
  using details::InteractiveMarkerInput<Eigen::Vector3d, false>::InteractiveMarkerInput;
};

} // namespace form

} // namespace mc_rtc_rviz
