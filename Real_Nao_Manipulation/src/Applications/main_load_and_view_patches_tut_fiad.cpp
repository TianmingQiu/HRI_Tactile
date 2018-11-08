#include <tum_ics_skin_descr/Patch/TfMarkerDataPatches.h>

#include <QApplication>
#include <QDebug>
#include <QVector>
#include <QFileInfo>
#include <QDir>

using namespace tum_ics_skin_descr;


int main( int argc, char** argv )
{
    ros::init(argc, argv, "load_and_view_patches_tut_fiad",
              ros::init_options::AnonymousName);
    ros::NodeHandle n;
    ros::Rate r(30);

    QApplication a(argc, argv);

    //    for(int i=0; i<argc; i++)
    //    {
    //        qDebug("%s",argv[i]);
    //    }

    if(argc != 2)
    {
        qCritical("Invalid number of arguments: %d",argc);
        return -1;
    }

    QString configFilePath = QString(argv[1]);
    QFileInfo fi(configFilePath);
    if(!fi.absoluteDir().exists())
    {
        qCritical("Invalid path '%s'",configFilePath.toAscii().data());
        return -1;
    }

    Patch::TfMarkerDataPatches tfPatches;

    if(!tfPatches.loadFromParam(configFilePath,"~patch_list"))
    {
        return -1;
    }
    tfPatches.createDataConnections();
    tfPatches.enableDataConnctions();

    for(int i=0; i<tfPatches.numberOfPatches(); i++)
    {
        Patch::TfMarkerDataPatch* p = tfPatches.patch(i);
        qDebug("%s",p->briefInfo().toAscii().data());
        p->setLedColor(Skin::LedColor::Blue);
    }

    while(ros::ok())
    {
        tfPatches.publish(ros::Duration(1.0));

        QApplication::processEvents();
        ros::spinOnce();
        r.sleep();
    }

    qDebug("exit");

    return 0;
}
