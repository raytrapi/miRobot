#include <iostream>

#include <QtWidgets/QApplication>
#include <QtWidgets/QDialog>
#ifdef WIN32
#include <windows.h>
#endif
#include "mainDialog.h"

QCoreApplication* createApplication(int &argc, char *argv[]){
    for (int i = 1; i < argc; ++i)
        if (!qstrcmp(argv[i], "-no-gui"))
            return new QCoreApplication(argc, argv);
    return new QApplication(argc, argv);
}
#ifdef WIN32
void Stealth(bool ocultar){
 HWND Stealth;
 AllocConsole();
 Stealth = FindWindowA("ConsoleWindowClass", NULL);
 ShowWindow(Stealth,(ocultar?0:1));
}
#endif
MainDialog *dialog;

int main(int argc, char* argv[]){

    QScopedPointer<QCoreApplication> app(createApplication(argc, argv));

    dialog = new MainDialog;
    dialog->show();
    //return app->exec();/**/
    std::cout<<"llego 1\r\n";
#ifdef WIN32
    //Stealth(true);
#endif
    //QApplication app2(argc, argv);
    /*QWidget *window=new QWidget;
    window->setWindowTitle("Hola");
    window->show();*/
    int res=app->exec();
#ifdef WIN32
    //Stealth(false);
#endif
    std::cout<<"llego 1\r\n";

    return res;
}



