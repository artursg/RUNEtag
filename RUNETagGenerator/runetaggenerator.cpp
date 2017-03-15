#include "runetaggenerator.h"
#include <QGraphicsEllipseItem>
#include <QPrinter>
#include <QPrintDialog>
#include <QFileDialog>
#include <QFile>
#include <QString>
#include <QMessageBox>
#include <QThread>
#include <fstream>

#include <time.h>
#include <vector>
#include <cassert>


#include <MarkPoint.h>

#include "coding.h"


#define M_PI 3.141592
#define GAP_FACTOR 1.3
#define RADIUS_RATIO 17.8
#define ELLYSIZE (1/RADIUS_RATIO)
#define NUM_LAYERS 3



template <typename T>
inline void showEllipsesInScene( const MarkPointSet<T>& point_set, QGraphicsScene& scene ) {

	typename std::vector< MarkPoint<T> >::const_iterator it = point_set.begin();
	
	QBrush brush;
	brush.setColor(Qt::black);
	brush.setStyle( Qt::SolidPattern );
	
	unsigned int i=0;
	while( it != point_set.end() ) {
		if( it->enabled ) {
            QGraphicsEllipseItem* ellipse_item = scene.addEllipse( QRectF(it->coords.real()-it->size, -it->coords.imag()-it->size, it->size*2.0, it->size*2.0), Qt::NoPen, brush );
		}
		it++;
		i++;
        
        /*
		if( i>3 )
			return;*/
	}
}


RUNETagGenerator::RUNETagGenerator(QGraphicsScene& _scene, QWidget *parent )
    : scene(_scene), QMainWindow(parent), curr_tag_idx(-1)
{
    ui.setupUi(this);
    
}


RUNETagGenerator::~RUNETagGenerator()
{
}

void RUNETagGenerator::on_actionExit_triggered()
{
    exit(0);
}

void RUNETagGenerator::on_actionLoad_tags_triggered()
{
    tagsdb.clear();
    curr_tag_idx = -1;
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open Tags definition file"), "",  tr("Files (*.*)"));
    
    // Load tags file
    std::string file_string( fileName.toLatin1().constData() );
    std::ifstream ifs(file_string.c_str());
    if( !TagCode::load_tags( ifs, tagsdb ) )
    {
        QMessageBox::warning(0, QString("Error"), QString("Unable to load tags"));
    }

    ifs.close();

    ui.crTextEdit->appendPlainText( QString::number( tagsdb.size() ) + " tags loaded.");

    
    alpha = ELLYSIZE * 2.0 * GAP_FACTOR;
    num_slots_for_layer = floor( 2.0 * M_PI / alpha );
    const int num_layers = NUM_LAYERS;
    num_slots = num_slots_for_layer*num_layers;
    alpha = 2.0 * M_PI / num_slots_for_layer;
    mark_point_set.clear();
    

    for( int i=0; i<num_slots_for_layer; i++ ) {
        for( int currLayer = 0; currLayer < num_layers; currLayer++ ) {
            const double angle = alpha * i;
            const double radius = 1.0 * (  (num_layers+currLayer+1)/(double)(num_layers*2.0)  );
            MarkPoint<double> curr_ellipse( radius*cos(angle), radius*sin(angle), angle, ELLYSIZE*radius, currLayer );
            curr_ellipse.enabled = false;
            mark_point_set.addPoint( curr_ellipse );
        } 
    }

}


void RUNETagGenerator::createMark() 
{

    scene.clear();
    scene.setSceneRect( -1.0-ELLYSIZE,-1.0-ELLYSIZE,2.0+ELLYSIZE*2.0,2.0+ELLYSIZE*2.0);
    //ui.crTextEdit->clear();

    if( curr_tag_idx >= tagsdb.size() )
        return;
    
    ui.crTextEdit->appendPlainText( "Radius ratio: " + QString::number( RADIUS_RATIO ) );
    ui.crTextEdit->appendPlainText( "Ellipse size (at external layer): " + QString::number( ELLYSIZE*ui.markerSize->value() ) + " mm.");
    ui.crTextEdit->appendPlainText( "Num slots: " + QString::number( num_slots ) );

    

    ui.crTextEdit->appendPlainText( "Code: " );
    size_t kk=0;
    for( unsigned int i=0; i<43; i++ ) {
        QString byn=QString::number(tagsdb[curr_tag_idx].bcode[kk])+QString::number(tagsdb[curr_tag_idx].bcode[kk+1])+QString::number(tagsdb[curr_tag_idx].bcode[kk+2]);
        ui.crTextEdit->appendPlainText( QString::number( tagsdb[curr_tag_idx].code[i]+1 ) + QString(" [ ") + byn + QString(" ] ") );
        kk+=3;
    }

    
	size_t n_ellipses=0;
    for( unsigned int i=0; i<129; i++ ) 
    {
        mark_point_set.pointAt( i ).enabled = tagsdb[curr_tag_idx].bcode[i];
		n_ellipses += mark_point_set.pointAt( i ).enabled ? 1 : 0;
    }
	ui.crTextEdit->appendPlainText( QString::number( n_ellipses ) + " total dots" );
    

    /*
    if( !ui.fullMarkerCheckBox->isChecked() ) {
        if( ui.numLayers->value() == 3 ) {
            ui.crTextEdit->appendPlainText( "Using code: " + QString::number( currcode_129 ) );
            mark_point_set.sort();
            for( unsigned int i=0; i<129; i++ ) {
                mark_point_set.pointAt( i ).enabled = codes_129[ currcode_129 ][i];
            }
            ui.markerName->setText("R129-" + QString::number( currcode_129 ) );
            max_score = maxscore_129;
        }
        if( ui.numLayers->value() == 1 ) {
            ui.crTextEdit->appendPlainText( "Using code: " + QString::number( currcode_43 ) );
            mark_point_set.sort();
            for( unsigned int i=0; i<43; i++ ) {
                mark_point_set.pointAt( i ).enabled = codes_43[ currcode_43 ][i];
            }
            ui.markerName->setText("R43-" + QString::number( currcode_43 ) );
            max_score = maxscore_43;
        }
    }
    */

#ifdef RANDOM_CODE
    if( !ui.fullMarkerCheckBox->isChecked() ) {
        int num_enabled = num_slots;

        while( num_enabled > 50 ) {
            unsigned int index = rand() % num_slots;
            if( mark_point_set.pointAt( index ).enabled ) {
                mark_point_set.pointAt( index ).enabled = false;
                --num_enabled;
            }
        }
    }
    mark_point_set.sort();
#endif

    //assert( mark_point_set.numPoints() == ui.numEllipses->value() );
    //numberEllipsesInScene( mark_point_set, scene );

    showEllipsesInScene( mark_point_set, scene);

	QPen r;
	r.setCosmetic(false);
	r.setWidthF(0.005);
	r.setColor(Qt::red);
	QPen g;
	g.setCosmetic(false);
	g.setWidthF(0.005);
	g.setColor(Qt::green);

    if( ui.showAxisCheckBox->isChecked() ) {
        scene.addLine( QLineF(0.0,0.0,1.0,0.0), r );
        scene.addLine( QLineF(0.0,0.0,0.0,-0.5), r );
    }

    if( ui.showSlotsCheckBox->isChecked() ) {
        for( int currLayer = 0; currLayer < NUM_LAYERS; currLayer++ ) {
            for( int i=0; i<num_slots_for_layer; i++ ) {
                const double angle = alpha * i + alpha/2.0;
                scene.addLine( QLineF(0.0,0.0,cos(angle), sin(angle)), g );
            }

        }
    }

}


void RUNETagGenerator::renderScene()
{
    createMark();
    ui.graphicsView->setScene( &scene );
    ui.graphicsView->show();
    ui.graphicsView->fitInView(scene.sceneRect(), Qt::KeepAspectRatio );
}


void RUNETagGenerator::on_nextCodeButton_clicked() 
{
	if( tagsdb.empty() )
		return;

    curr_tag_idx = (curr_tag_idx+1)%tagsdb.size();
    ui.tagIndex->setText( QString::number(tagsdb[curr_tag_idx].idx));
    createMark();
    renderScene();
}

void RUNETagGenerator::on_showAxisCheckBox_stateChanged( int state ) 
{
    renderScene();
}
void RUNETagGenerator::on_showSlotsCheckBox_stateChanged( int state ) 
{
    renderScene();
}
void RUNETagGenerator::on_fullMarkerCheckBox_stateChanged( int state ) 
{
    renderScene();
}

void RUNETagGenerator::on_actionAbout_triggered() 
{
    QMessageBox::about ( this, tr("About"), tr("RUNETag Generator v.2") ) ;
}

void RUNETagGenerator::on_actionPrint_triggered()
{

    ui.statusBar->showMessage( tr("Exporting...") );
    QString filename = QFileDialog::getSaveFileName( this, "Export to PDF", ui.markerName->text(), tr("PDF File (*.pdf)") );

    QFile fl( filename );
    if( fl.exists() )
        fl.remove();

    QPrinter printer( QPrinter::HighResolution );
    printer.setPaperSize( QPrinter::A4 );
    printer.setOrientation( QPrinter::Portrait );


    printer.setPageSize( QPrinter::A4 );
    printer.setPageMargins( 0,0,0,0, QPrinter::Millimeter );

    printer.setOutputFileName( filename );

    QSizeF page_mil = printer.paperSize( QPrinter::Millimeter );
    QSizeF pagePx = printer.paperSize(QPrinter::DevicePixel);
    float convW = pagePx.width() / page_mil.width();
    float convH = pagePx.height() / page_mil.height();

    QPainter painter(&printer);

    float canv_size = 1.0+ELLYSIZE;

    double m_width = canv_size*ui.markerSize->value()*convW;
    double m_height = canv_size*ui.markerSize->value()*convH;
    QRectF target( (pagePx.width()-m_width)/2, (pagePx.height()-m_height)/2, m_width, m_height );
    scene.render( &painter, target, scene.sceneRect(), Qt::KeepAspectRatio );

    ui.statusBar->showMessage( tr("Mark successfully exported") );
	

    //Export descriptor
    QString desc_filename = QFileDialog::getSaveFileName( this, "Export descriptor", ui.markerName->text(),tr("Text File (*.txt)") );

    std::string file_string( desc_filename.toLatin1().constData() );

    std::ofstream ofs( file_string.c_str() );
    if( ofs.fail() ) {
        return;
    }

    ofs << "RUNE_direct" << std::endl;
    ofs << std::string( ui.markerName->text().toLatin1().constData() ) << std::endl;
    ofs << (ui.markerSize->value() / 2.0) << std::endl;
    ofs << "mm" << std::endl;
    ofs << mark_point_set.numPoints() << std::endl;
    ofs << RADIUS_RATIO << std::endl;
    ofs << NUM_LAYERS << std::endl;
    ofs << GAP_FACTOR << std::endl;
    ofs << "-1" << std::endl;
    ofs << tagsdb[curr_tag_idx].idx << std::endl;

    for( unsigned int i=0; i<mark_point_set.numPoints(); i++ ) {
        if( mark_point_set.pointAt(i).enabled ) {
            //
            //  Circle quadratic form
            //   1   0   -cx
            //   0   1   -cy
            // -cx  -cy  cx^2 + cy^2 - r^2
            //
            const double radius_big = ui.markerSize->value() / 2.0;
            const double radius = mark_point_set.pointAt(i).size * radius_big;
            const double cx = mark_point_set.pointAt(i).coords.real()*radius_big;
            const double cy = mark_point_set.pointAt(i).coords.imag()*radius_big;
            const double k = cx*cx + cy*cy - radius*radius;

            ofs << "1 ";
            ofs << "1.0 0.0 " << (-cx) << " "; //first row
            ofs << "0.0 1.0 " << (-cy) << " "; //second row
            ofs << (-cx) << " " << (-cy) << " " << k << std::endl; //third row

        } else {
            ofs << "0" << std::endl;
        }
    }

    ofs.close();

}
