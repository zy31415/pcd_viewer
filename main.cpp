#include "pcdviewermainwindow.h"
#include <QApplication>
#include <QMainWindow>

int main (int argc, char *argv[])
{
  QApplication a (argc, argv);
  PCDViewerMainWindow w;
  w.show ();

  return a.exec ();
}
