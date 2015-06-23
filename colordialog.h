#ifndef COLORDIALOG_H
#define COLORDIALOG_H

#include <QDialog>

namespace Ui {
  class ColorDialog;
}

class ColorDialog : public QDialog {
    Q_OBJECT

private:
    Ui::ColorDialog* cdialog;

public:
    ColorDialog(QWidget *parent = 0);
    virtual ~ColorDialog();

    int get_color_changing_axis();
    int get_look_up_table();
};

#endif // COLORDIALOG_H
