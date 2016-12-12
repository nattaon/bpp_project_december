#include "MainUI.h"
#include "DataProcess.h"
#include "ViewerWindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

	HWND consoleWindow = GetConsoleWindow();
	SetWindowPos(consoleWindow, 0, 1000, 0, 600, 600, SWP_NOZORDER);




    DataProcess d; //=call DataProcess::DataProcess()

	ViewerWindow v; //=call ViewerWindow::ViewerWindow()
    v.SetDataProcess(&d);

    MainUI mainui;
    mainui.SetDataProcess(&d);
    mainui.SetViewerWindow(&v);
    mainui.show();
	mainui.move(600, 340);

    return a.exec();
}
