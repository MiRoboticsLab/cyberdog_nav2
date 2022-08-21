#ifndef NAV_PUSH_BTN_H
#define NAV_PUSH_BTN_H

#include <QComboBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QSizePolicy>
#include <QPushButton>
#include <QTextEdit>

class NavPushBtn : public QHBoxLayout
{
  Q_OBJECT

public:
  NavPushBtn(QWidget * parent = nullptr);
  ~NavPushBtn();
  void setEnabled(bool enable);

Q_SIGNALS:
  bool clicked();

private:
  QPushButton * action_button_;
  QTextEdit * info_edit_;
};

#endif
