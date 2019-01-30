#include "ArrayInputWidget.h"

namespace mc_rtc_rviz
{

ArrayInputWidget::ArrayInputWidget(const ClientWidgetParam & param, const std::vector<std::string> & labels)
: ClientWidget(param)
{
  auto mainLayout = new QVBoxLayout(this);

  labels_layout_ = new QHBoxLayout();
  mainLayout->addLayout(labels_layout_);
  labels_layout_->addWidget(new QLabel(name().c_str()));
  lock_button_ = new QPushButton("EDIT");
  lock_button_->setCheckable(true);
  labels_layout_->addWidget(lock_button_);
  connect(lock_button_, SIGNAL(toggled(bool)), this, SLOT(lock_toggled(bool)));
  edits_layout_ = new QGridLayout();
  mainLayout->addLayout(edits_layout_);
  if(labels.size())
  {
    edits_row_ = 1;
    int col = 0;
    for(const auto & l : labels)
    {
      edits_layout_->addWidget(new QLabel(l.c_str()), 0, col++, Qt::AlignCenter);
    }
  }
}

void ArrayInputWidget::update(const Eigen::VectorXd & data)
{
  if(lock_button_->isChecked())
  {
    return;
  }
  if(data.size() != edits_.size())
  {
    auto old_size = edits_.size();
    edits_.resize(data.size());
    for(size_t i = old_size; i < edits_.size(); ++i)
    {
      edits_[i] = new QLineEdit(this);
      edits_[i]->setReadOnly(true);
      auto validator = new QDoubleValidator(this);
      validator->setLocale(QLocale::C);
      edits_[i]->setValidator(validator);
      connect(edits_[i], SIGNAL(returnPressed()), this, SLOT(edit_return_pressed()));
      edits_layout_->addWidget(edits_[i], edits_row_, i);
    }
  }
  for(size_t i = 0; i < edits_.size(); ++i)
  {
    edits_[i]->setText(QString::number(data(i)));
    edits_[i]->setCursorPosition(0);
  }
}

void ArrayInputWidget::lock_toggled(bool unlocked)
{
  if(unlocked)
  {
    lock_button_->setText("SEND");
    for(auto e : edits_)
    {
      e->setReadOnly(false);
    }
  }
  else
  {
    lock_button_->setText("EDIT");
    std::vector<double> v;
    v.reserve(edits_.size());
    for(auto e : edits_)
    {
      v.push_back(e->text().toDouble());
      e->setReadOnly(true);
    }
    client().send_request(id(), v);
  }
}

void ArrayInputWidget::edit_return_pressed()
{
  if(lock_button_->isChecked())
  {
    lock_button_->toggle();
  }
}

QPushButton * ArrayInputWidget::showHideButton()
{
  QPushButton * ret = new QPushButton("Hide");
  labels_layout_->addWidget(ret);
  return ret;
}

} // namespace mc_rtc_rviz
