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


#include "nav_combo_box.hpp"
#include "protocol/action/navigation.hpp"

NavComboBox::NavComboBox(QWidget *)   // NOLINT
: QGridLayout()
{
  gaol_method_list_ = new QComboBox();
  gaol_method_list_->addItem("NAVIGATION_GOAL_TYPE_UNKNWON");
  gaol_method_list_->addItem("NAVIGATION_GOAL_TYPE_AB");
  gaol_method_list_->addItem("NAVIGATION_GOAL_TYPE_FOLLOW");
  gaol_method_list_->addItem("NAVIGATION_GOAL_TYPE_MAPPING");
  gaol_method_list_->addItem("NAVIGATION_GOAL_TYPE_STOP_MAPPING");

  QHBoxLayout * l1 = new QHBoxLayout;

  QLabel * action_label = new QLabel("Navigation Action:");
  QSizePolicy spLeft(QSizePolicy::Preferred, QSizePolicy::Preferred);
  spLeft.setHorizontalStretch(1);
  action_label->setSizePolicy(spLeft);
  l1->addWidget(action_label);
  // this->addWidget(action_label, 0, 0);

  QSizePolicy spRight(QSizePolicy::Preferred, QSizePolicy::Preferred);
  spRight.setHorizontalStretch(1);
  gaol_method_list_->setSizePolicy(spRight);
  l1->addWidget(gaol_method_list_);
  // this->addWidget(gaol_method_list_);
  this->addLayout(l1, 0, 0);

  // QHBoxLayout* l2 = new QHBoxLayout;

  // QSizePolicy sp_left(QSizePolicy::Preferred, QSizePolicy::Preferred);
  // sp_left.setHorizontalStretch(1);
  // info_edit_ = new QTextEdit;
  // info_edit_->setSizePolicy(sp_left);
  // info_edit_->setReadOnly(true);
  // l2->addWidget(info_edit_);

  // QSizePolicy sp_right(QSizePolicy::Preferred, QSizePolicy::Preferred);
  // sp_right.setHorizontalStretch(1);
  // action_button_ = new QPushButton("Start");
  // action_button_->setSizePolicy(sp_right);
  // l2->addWidget(action_button_);
  // this->addLayout(l2, 1, 0);

  connect(gaol_method_list_, SIGNAL(currentIndexChanged(int)), SLOT(reemit_signal(int)));
}

NavComboBox::~NavComboBox() {}

void NavComboBox::reemit_signal(int newvalue)
{
  int goal_number = 0;
  switch (newvalue) {
    case (0):
      goal_number = protocol::action::Navigation::Goal::NAVIGATION_TYPE_UNKNWON;
      break;
    case (1):
      goal_number = protocol::action::Navigation::Goal::NAVIGATION_TYPE_START_AB;
      break;
    case (2):
      goal_number = protocol::action::Navigation::Goal::NAVIGATION_TYPE_START_FOLLOW;
      break;
    case (3):
      goal_number = protocol::action::Navigation::Goal::NAVIGATION_TYPE_START_MAPPING;
      break;
    case (4):
      goal_number = protocol::action::Navigation::Goal::NAVIGATION_TYPE_STOP_MAPPING;
      break;
    default:
      goal_number = protocol::action::Navigation::Goal::NAVIGATION_TYPE_UNKNWON;
      break;
  }
  Q_EMIT valueChanged(goal_number);
}
