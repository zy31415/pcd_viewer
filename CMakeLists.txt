cmake_minimum_required (VERSION 2.6 FATAL_ERROR)

project      (pcd_viewer)
find_package (Qt4 REQUIRED)
find_package (VTK REQUIRED)
find_package (PCL 1.8.0 REQUIRED)

include_directories (${PCL_INCLUDE_DIRS})
link_directories    (${PCL_LIBRARY_DIRS})
add_definitions     (${PCL_DEFINITIONS})

set  (project_SOURCES main.cpp pclviewer.cpp colordialog.cpp triangulationdialog.cpp
		triangulation_meshes.cpp tourdialog.cpp boundingbox.cpp	worker.cpp)

set  (project_HEADERS pclviewer.h colordialog.h triangulationdialog.h
		triangulation_meshes.h tourdialog.h boundingbox.h worker.h)

set  (project_FORMS   pclviewer.ui colordialog.ui triangulationdialog.ui tourdialog.ui)
set  (VTK_LIBRARIES   vtkRendering vtkGraphics vtkHybrid QVTK)

QT4_WRAP_CPP (project_HEADERS_MOC   ${project_HEADERS})
QT4_WRAP_UI  (project_FORMS_HEADERS ${project_FORMS})

INCLUDE         (${QT_USE_FILE})
ADD_DEFINITIONS (${QT_DEFINITIONS})

qt4_add_resources(UI_RESOURCES resources.qrc)

ADD_EXECUTABLE  (pcd_viewer ${project_SOURCES}
                                ${project_FORMS_HEADERS}
                                ${project_HEADERS_MOC}
								${UI_RESOURCES})

TARGET_LINK_LIBRARIES (pcd_viewer ${QT_LIBRARIES} ${PCL_LIBRARIES} ${VTK_LIBRARIES} pthread)

