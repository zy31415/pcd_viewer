#include <pcl/io/ply_io.h>

#include "colordialog.h"
#include "../build/ui_colordialog.h"

#include "pclviewer.h"

int ColorDialog::DEFAULT_POINT_SIZE = 2;

ColorDialog::ColorDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ColorDialog),
    point_size(DEFAULT_POINT_SIZE)
{
    ui->setupUi(this);

    pclViewer_ = (PCLViewer*)parentWidget();
    viewer_ = pclViewer_ -> getViewer();

    ui->lineEdit->setText(QString::number(point_size));

    //
    connect (ui->radioButton_x_2, SIGNAL(clicked ()), parent, SLOT(axisChosen ()));
    connect (ui->radioButton_y_2, SIGNAL(clicked ()), parent, SLOT(axisChosen ()));
    connect (ui->radioButton_z_2, SIGNAL(clicked ()), parent, SLOT(axisChosen ()));

    connect (ui -> radioButton_BlueRed_2, SIGNAL(clicked ()), parent, SLOT(lookUpTableChosen()));
    connect (ui -> radioButton_GreenMagenta_2, SIGNAL(clicked ()), parent, SLOT(lookUpTableChosen()));
    connect (ui -> radioButton_WhiteRed_2, SIGNAL(clicked ()), parent, SLOT(lookUpTableChosen()));
    connect (ui -> radioButton_GreyRed_2, SIGNAL(clicked ()), parent, SLOT(lookUpTableChosen()));
    connect (ui -> radioButton_Rainbow_2, SIGNAL(clicked ()), parent, SLOT(lookUpTableChosen()));

    connect (ui -> checkBox, SIGNAL(toggled(bool)), this, SLOT(onIfShowDataPoints()));

    connect (ui -> lineEdit, SIGNAL(editingFinished()), this, SLOT(onChangePointSize()));

    viewer_->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "cloud");
}

ColorDialog::~ColorDialog() {
    delete ui;
}

int ColorDialog::get_color_changing_axis() {
    // Only 1 of the button can be checked at the time (mutual exclusivity) in a group of radio buttons
    if (ui->radioButton_x_2->isChecked ())
    {
      PCL_INFO("x filtering chosen\n");
      return 0;
    }

    if (ui->radioButton_y_2->isChecked ()) {
        PCL_INFO("y filtering chosen\n");
        return 1;
    }

    PCL_INFO("z filtering chosen\n");
    return 2;
}


int ColorDialog::get_look_up_table() {
    // Only 1 of the button can be checked at the time (mutual exclusivity) in a group of radio buttons
    if (ui -> radioButton_BlueRed_2->isChecked ()) {
      PCL_INFO("Blue -> Red LUT chosen\n");
      return 0;
    }

    if (ui -> radioButton_GreenMagenta_2->isChecked ()) {
      PCL_INFO("Green -> Magenta LUT chosen\n");
      return 1;
    }

    if (ui -> radioButton_WhiteRed_2->isChecked ()) {
      PCL_INFO("White -> Red LUT chosen\n");
      return 2;
    }

    if (ui -> radioButton_GreyRed_2->isChecked ()) {
      PCL_INFO("Grey / Red LUT chosen\n");
      return 3;
    }

    PCL_INFO("Rainbow LUT chosen\n");
    return 4;
}

void ColorDialog::onIfShowDataPoints() {
    if (ui->checkBox->isChecked())
        pclViewer_->addPointsCloudToView();
    else
        pclViewer_->removePointsCloudFromView();
}

void ColorDialog::onChangePointSize() {
    point_size = ui->lineEdit->text().toInt();

    viewer_->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "cloud");
}


