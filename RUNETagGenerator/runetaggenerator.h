#ifndef RUNETAGGENERATOR_H
#define RUNETAGGENERATOR_H

#include <QMainWindow>
#include "ui_runetaggenerator.h"
#include <MarkPointSet.h>
#include "TagCode.h"

class RUNETagGenerator : public QMainWindow
{
    Q_OBJECT

public:
    RUNETagGenerator(QGraphicsScene& _scene, QWidget *parent = 0 );
    ~RUNETagGenerator();
    void renderScene();
    void createMark();

private:
    Ui::RUNETagGeneratorClass ui;
    QGraphicsScene& scene;
    MarkPointSetd mark_point_set;
    std::vector<long> curr_code;
    std::vector<TagCode> tagsdb;
    size_t curr_tag_idx;
    int num_slots;
    int num_slots_for_layer;
    double alpha;

private slots:
    void on_showAxisCheckBox_stateChanged( int state );
    void on_showSlotsCheckBox_stateChanged( int state );
    void on_fullMarkerCheckBox_stateChanged( int state );
    //void on_generateButton_clicked();
    void on_nextCodeButton_clicked();
    //void on_actionPrint_triggered();
    void on_actionExit_triggered();
    void on_actionAbout_triggered();
    void on_actionLoad_tags_triggered();
    void on_actionPrint_triggered();

};

#endif // RUNETAGGENERATOR_H
