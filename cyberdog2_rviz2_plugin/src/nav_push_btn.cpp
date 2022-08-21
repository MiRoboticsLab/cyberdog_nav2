#include "nav_push_btn.hpp"

NavPushBtn::NavPushBtn(QWidget *)
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
