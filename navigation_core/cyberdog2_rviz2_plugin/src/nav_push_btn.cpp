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


#include "nav_push_btn.hpp"

NavPushBtn::NavPushBtn(QWidget *)  // NOLINT
: QHBoxLayout()
{
  QSizePolicy sp_left(QSizePolicy::Preferred, QSizePolicy::Preferred);
  sp_left.setHorizontalStretch(1);
  info_edit_ = new QTextEdit;
  info_edit_->setSizePolicy(sp_left);
  info_edit_->setReadOnly(true);
  this->addWidget(info_edit_);

  QSizePolicy sp_right(QSizePolicy::Preferred, QSizePolicy::Preferred);
  sp_right.setHorizontalStretch(1);
  action_button_ = new QPushButton("Start");
  action_button_->setSizePolicy(sp_right);
  this->addWidget(action_button_);

  connect(action_button_, &QPushButton::clicked, [this](void) {Q_EMIT clicked();});
}

NavPushBtn::~NavPushBtn() {}

void NavPushBtn::setEnabled(bool enable)
{
  if (enable) {
    action_button_->setEnabled(true);
  } else {
    action_button_->setDisabled(true);
  }
}
