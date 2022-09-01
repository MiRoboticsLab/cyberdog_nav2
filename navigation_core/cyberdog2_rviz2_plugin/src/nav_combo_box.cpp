#include "nav_combo_box.hpp"
#include "protocol/action/navigation.hpp"

NavComboBox::NavComboBox(QWidget *)
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
      goal_number = protocol::action::Navigation::Goal::NAVIGATION_GOAL_TYPE_UNKNWON;
      break;
    case (1):
      goal_number = protocol::action::Navigation::Goal::NAVIGATION_GOAL_TYPE_AB;
      break;
    case (2):
      goal_number = protocol::action::Navigation::Goal::NAVIGATION_GOAL_TYPE_FOLLOW;
      break;
    case (3):
      goal_number = protocol::action::Navigation::Goal::NAVIGATION_GOAL_TYPE_MAPPING;
      break;
    case (4):
      goal_number = protocol::action::Navigation::Goal::NAVIGATION_GOAL_TYPE_STOP_MAPPING;
      break;
    default:
      goal_number = protocol::action::Navigation::Goal::NAVIGATION_GOAL_TYPE_UNKNWON;
      break;
  }
  Q_EMIT valueChanged(goal_number);
}
