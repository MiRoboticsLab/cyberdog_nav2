// Copyright (c) 2021 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#ifndef NAV_COMBO_BOX_HPP_
#define NAV_COMBO_BOX_HPP_

#include <QComboBox>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QSizePolicy>
#include <QPushButton>
#include <QTextEdit>

class NavComboBox : public QGridLayout
{
  Q_OBJECT

public:
  explicit NavComboBox(QWidget * parent = nullptr);
  ~NavComboBox();

Q_SIGNALS:
  void valueChanged(int newvalue);

private Q_SLOTS:
  void reemit_signal(int index);

private:
  QComboBox * gaol_method_list_;
  // QPushButton* action_button_;
  // QTextEdit* info_edit_;
};

#endif  // NAV_COMBO_BOX_HPP_
