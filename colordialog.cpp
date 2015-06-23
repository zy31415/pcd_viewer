#include <pcl/io/ply_io.h>

#include "colordialog.h"
#include "../build/ui_colordialog.h"

ColorDialog::ColorDialog(QWidget *parent) :
    QDialog(parent),
    cdialog(new Ui::ColorDialog)
{
    cdialog->setupUi(this);

    //
    connect (cdialog->radioButton_x_2, SIGNAL(clicked ()), parent, SLOT(axisChosen ()));
    connect (cdialog->radioButton_y_2, SIGNAL(clicked ()), parent, SLOT(axisChosen ()));
    connect (cdialog->radioButton_z_2, SIGNAL(clicked ()), parent, SLOT(axisChosen ()));

    connect (cdialog -> radioButton_BlueRed_2, SIGNAL(clicked ()), parent, SLOT(lookUpTableChosen()));
    connect (cdialog -> radioButton_GreenMagenta_2, SIGNAL(clicked ()), parent, SLOT(lookUpTableChosen()));
    connect (cdialog -> radioButton_WhiteRed_2, SIGNAL(clicked ()), parent, SLOT(lookUpTableChosen()));
    connect (cdialog -> radioButton_GreyRed_2, SIGNAL(clicked ()), parent, SLOT(lookUpTableChosen()));
    connect (cdialog -> radioButton_Rainbow_2, SIGNAL(clicked ()), parent, SLOT(lookUpTableChosen()));

}

ColorDialog::~ColorDialog() {
    delete cdialog;
}

int ColorDialog::get_color_changing_axis() {
    // Only 1 of the button can be checked at the time (mutual exclusivity) in a group of radio buttons
    if (cdialog->radioButton_x_2->isChecked ())
    {
      PCL_INFO("x filtering chosen\n");
      return 0;
    }

    if (cdialog->radioButton_y_2->isChecked ()) {
        PCL_INFO("y filtering chosen\n");
        return 1;
    }

    PCL_INFO("z filtering chosen\n");
    return 2;
}


int ColorDialog::get_look_up_table() {
    // Only 1 of the button can be checked at the time (mutual exclusivity) in a group of radio buttons
    if (cdialog -> radioButton_BlueRed_2->isChecked ()) {
      PCL_INFO("Blue -> Red LUT chosen\n");
      return 0;
    }

    if (cdialog -> radioButton_GreenMagenta_2->isChecked ()) {
      PCL_INFO("Green -> Magenta LUT chosen\n");
      return 1;
    }

    if (cdialog -> radioButton_WhiteRed_2->isChecked ()) {
      PCL_INFO("White -> Red LUT chosen\n");
      return 2;
    }

    if (cdialog -> radioButton_GreyRed_2->isChecked ()) {
      PCL_INFO("Grey / Red LUT chosen\n");
      return 3;
    }

    PCL_INFO("Rainbow LUT chosen\n");
    return 4;
}
