#include "runetaggenerator.h"
#include <QApplication>
#include <QGraphicsScene>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    QGraphicsScene scene;
    scene.setSceneRect( 0,0,430,430);
    RUNETagGenerator w(scene);
    w.show();
    w.renderScene();
    return a.exec();
}
