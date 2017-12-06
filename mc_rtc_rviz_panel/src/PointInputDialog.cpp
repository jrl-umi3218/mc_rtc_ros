#include "PointInputDialog.h"

#include <QDoubleValidator>

#include <iostream>

PointInputDialog::PointInputDialog(const std::string & name, const std::vector<std::string> & labels_names, bool editable, bool stacked, request_t request, double min, double max)
: request(request),
  layout(new QGridLayout()),
  name_label(new QLabel(name.c_str())),
  button(new QPushButton("🔒"))
{
  layout->addWidget(name_label, 0, 0);
  size_t N = labels_names.size() > 1 ? labels_names.size() : 1;
  if(editable)
  {
    button->setSizePolicy({QSizePolicy::Policy::Fixed, QSizePolicy::Policy::Preferred});
    button->setCheckable(true);
    layout->addWidget(button, 0, stacked ? 1 : N == 1 ? 2 : N - 1, Qt::AlignRight);
    connect(button, &QPushButton::toggled,
            this, &PointInputDialog::toggled);
  }
  labels.resize(N, nullptr);
  data.resize(N, nullptr);
  if(labels_names.size() > 1)
  {
    for(size_t i = 0; i < labels_names.size(); ++i)
    {
      const std::string & n = labels_names[i];
      labels[i] = new QLabel(n.c_str());
      data[i] = new QLineEdit();
      data[i]->setValidator(new QDoubleValidator(min, max, 100));
      data[i]->setReadOnly(true);
      connect(data[i], &QLineEdit::returnPressed,
              this, &PointInputDialog::returnPressed);
      if(!stacked)
      {
        layout->addWidget(labels[i], 1, i, Qt::AlignCenter);
        layout->addWidget(data[i], 2, i, Qt::AlignCenter);
      }
      else
      {
        layout->addWidget(labels[i], i+1, 0);
        layout->addWidget(data[i], i+1, 1);
      }
    }
  }
  else
  {
    data[0] = new QLineEdit();
    data[0]->setValidator(new QDoubleValidator(min, max, 100));
    data[0]->setReadOnly(true);
    connect(data[0], &QLineEdit::returnPressed,
            this, &PointInputDialog::returnPressed);
    layout->addWidget(data[0], 0, 1);
  }
  setLayout(layout);
}

void PointInputDialog::update(const mc_rtc::Configuration & data_in)
{
  if(data.size() != 1 && data_in.size() != data.size())
  {
    std::cerr << "PointInputDialog given data of invalid size " << data_in.size() << " (expected: " << data.size() << ")" << std::endl;
    return;
  }
  if(button->isChecked()) return;
  if(data.size() > 1)
  {
    for(size_t i = 0; i < data.size(); ++i)
    {
      data[i]->setText(data_in[i].dump().c_str());
      data[i]->setCursorPosition(0);
    }
  }
  else
  {
    data[0]->setText(data_in.dump().c_str());
    data[0]->setCursorPosition(0);
  }
}

void PointInputDialog::toggled(bool checked)
{
  if(!checked)
  {
    mc_rtc::Configuration data_out;
    if(data.size() > 1)
    {
      data_out = data_out.array("data", data.size());
      for(size_t i = 0; i < data.size(); ++i)
      {
        data_out.push(data[i]->text().toDouble());
        data[i]->setReadOnly(true);
      }
      request(data_out);
    }
    else
    {
      data_out.add("data", data[0]->text().toDouble());
      request(data_out("data"));
    }
    button->setText("🔒");
  }
  else
  {
    for(size_t i = 0; i < data.size(); ++i)
    {
      data[i]->setReadOnly(false);
    }
    button->setText("🔓");
  }
}

void PointInputDialog::returnPressed()
{
  if(button->isChecked())
  {
    button->toggle();
  }
}
