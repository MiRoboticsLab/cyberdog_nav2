#ifndef NAV_COMBO_BOX_H
#define NAV_COMBO_BOX_H

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
  NavComboBox(QWidget * parent = nullptr);
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

#endif
