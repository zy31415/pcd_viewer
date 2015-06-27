#include <pcl/io/ply_io.h>

#include "colordialog.h"
#include "../build/ui_colordialog.h"

#include "pcdviewermainwindow.h"


ColorDialog::ColorDialog(DataModel* data_, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ColorDialog),
    data_(data_)
{
    ui->setupUi(this);

    ui->lineEdit->setText(QString::number(data_->getPointSize()));

    if (data_->getIfShowDataPoints())
        ui->checkBox_if_show_data_points->setChecked(true);
    else
        ui->checkBox_if_show_data_points->setChecked(false);

    switch (data_->getColorAxis())
    {
        case 0:
            ui->radioButton_x->setChecked(true);
            break;
        case 1:
            ui->radioButton_y->setChecked(true);
            break;
        default:
            ui->radioButton_z->setChecked(true);
            break;
    }

    switch (data_->getColorLookUpTable())
    {
        case 0:
            ui->radioButton_BlueRed->setChecked(true);
            break;
        case 1:
            ui->radioButton_GreenMagenta->setChecked(true);
            break;
        case 2:
            ui->radioButton_WhiteRed->setChecked(true);
            break;
        case 3:
            ui->radioButton_GreyRed->setChecked(true);
            break;
        default:
            ui->radioButton_Rainbow->setChecked(true);
            break;
    }

    connect (ui->radioButton_x, SIGNAL(clicked ()), this, SLOT(onAxisChosen()));
    connect (ui->radioButton_y, SIGNAL(clicked ()), this, SLOT(onAxisChosen()));
    connect (ui->radioButton_z, SIGNAL(clicked ()), this, SLOT(onAxisChosen()));

    connect (ui -> radioButton_BlueRed, SIGNAL(clicked ()), this, SLOT(onLookUpTableChosen()));
    connect (ui -> radioButton_GreenMagenta, SIGNAL(clicked ()), this, SLOT(onLookUpTableChosen()));
    connect (ui -> radioButton_WhiteRed, SIGNAL(clicked ()), this, SLOT(onLookUpTableChosen()));
    connect (ui -> radioButton_GreyRed, SIGNAL(clicked ()), this, SLOT(onLookUpTableChosen()));
    connect (ui -> radioButton_Rainbow, SIGNAL(clicked ()), this, SLOT(onLookUpTableChosen()));

    connect (ui->checkBox_if_show_data_points, SIGNAL(toggled(bool)), this, SLOT(onIfShowDataPoints()));
    connect (ui->lineEdit, SIGNAL(editingFinished()), this, SLOT(onChangePointSize()));

}

ColorDialog::~ColorDialog() {
    delete ui;
}

void ColorDialog::onIfShowDataPoints()
{
    data_->setIfShowDataPoints(
                ui->checkBox_if_show_data_points->isChecked());
}

void ColorDialog::onChangePointSize() {
    int point_size = ui->lineEdit->text().toInt();
    data_ -> setPointSize(point_size);
}


void ColorDialog::onAxisChosen() {
    int filtering_axis_;
    if (ui->radioButton_x->isChecked ())
    {
        PCL_INFO("x filtering chosen\n");
        filtering_axis_ = 0;
    }
    else if (ui->radioButton_y->isChecked ())
    {
        PCL_INFO("y filtering chosen\n");
        filtering_axis_ = 1;
    }
    else
    {
        PCL_INFO("z filtering chosen\n");
        filtering_axis_ = 2;
    }
    data_->setColorAxis(filtering_axis_);
}

void ColorDialog::onLookUpTableChosen() {
    int color_mode_ = 0;
    if (ui->radioButton_BlueRed->isChecked())
    {
        PCL_INFO("Blue -> Red LUT chosen\n");
        color_mode_ = 0;
    }
    else if (ui->radioButton_GreenMagenta->isChecked ())
    {
        PCL_INFO("Green -> Magenta LUT chosen\n");
        color_mode_ = 1;
    }
    else if (ui->radioButton_WhiteRed->isChecked ())
    {
        PCL_INFO("White -> Red LUT chosen\n");
        color_mode_ = 2;
    }
    else if (ui->radioButton_GreyRed->isChecked ())
    {
        PCL_INFO("Grey / Red LUT chosen\n");
        color_mode_ = 3;
    }
    else
    {
        PCL_INFO("Rainbow LUT chosen\n");
        color_mode_ = 4;
    }
    data_->setColorLookUpTable(color_mode_);
}
