#include <tum_ics_skin_descr/Patch/TfMarkerDataPatch.h>

#include <QApplication>
#include <QDebug>
#include <QVector>
#include <QFileInfo>
#include <QDir>

using namespace tum_ics_skin_descr;


int main( int argc, char** argv )
{
    ros::init(argc, argv, "load_and_view_patch_tut_fiad",
              ros::init_options::AnonymousName);
    ros::NodeHandle n;
    ros::Rate r(30);

    QApplication a(argc, argv);

//    for(int i=0; i<argc; i++)
//    {
//        qDebug("%s",argv[i]);
//    }

    if(argc != 4)
    {
        qCritical("Invalid number of arguments: %d",argc);
        return -1;
    }

    // Get the parameters from the input arguments
    bool ok;
    int patchId = QString(argv[2]).toUInt(&ok);
    if(!ok)
    {
        qCritical("Invalid patch id '%s'",argv[2]);
        return -1;
    }

    QString fileDiagEnabledStr(argv[3]);
    if(fileDiagEnabledStr != "true" &&
            fileDiagEnabledStr != "false")
    {
        qCritical("Invalid file dialog option");
        return -1;
    }

    bool fileDiagEnabled = true;
    if(fileDiagEnabledStr == "false")
    {
        fileDiagEnabled = false;
    }

    QString configFilePath = QString(argv[1]);
    QFileInfo fi(configFilePath);
    if(!fi.absoluteDir().exists())
    {
        qCritical("Invalid path '%s'",configFilePath.toAscii().data());
        return -1;
    }

    if(!fi.exists())
    {
        qCritical("Invalid path or file '%s'",configFilePath.toAscii().data());
        return -1;
    }

    // This class is the container for the skin patch.
    Patch::TfMarkerDataPatch patch;

    if(fi.isDir() || fileDiagEnabled == true)
    {
        if(!patch.loadDialog(configFilePath,patchId))
        {
            return -1;
        }
    }
    else
    {
        if(!patch.load(configFilePath,patchId))
        {
            return -1;
        }
    }

    if(patch.isUndefined())
    {
        qCritical("Loaded patch is undefined");
        return -1;
    }

    qDebug("%s",patch.briefInfo().toAscii().data());

    // Create the ROS topics of the skin patch and its cells
    patch.createDataConnection();

    // Enables the publishers in the driver and subscribes to these publishers. In this way the cell data will be available
    // in your application (here!)
    patch.enableDataConnection();

    // With this function you can define the base frame for the patch (TF name), e.g. a specific joint in a robot
    patch.setBaseFrame("/world");

    //You can define the Homogeneous Transformation between the base frame and the patch, e.g. the patch1 w.r.t /world
    patch.setPose(Eigen::Affine3d::Identity());


    bool oneShot=true;

    while(ros::ok())
    {
        // This function publishes the visualization MArkers and the TFs of the skin patch and its cells.
        // This function allows to visualize the patch in RViz
        patch.publish(ros::Duration(1.0));


        // The API provides several access functions to request information of the patch and the cells, e.g.
        //  1) access the data of cell with id 4
        double force1 = patch.dataFromId(4).force.at(0);

        ROS_INFO_STREAM("Force 1 on Cell ID 4: "<<force1);


        //  2) acces transformation of cell with id 3 w.r.t the base frame
        //      Get the index of cell with id 3
        int cell3_ind = patch.index(3);

        Eigen::Affine3d T_cell3_world = patch.tf_cell_base().at(cell3_ind);


        //  3) Change the color of cell 4
        if(oneShot)
        {
            patch.setLedColor(Skin::Cell::LedColor::Red, 4);
            oneShot = false;
        }


        //  etc. See TfMarkerDataPatch.h for more information on this.


        // We use QT events which need to be triggered
        // QApplication::processEvents();
        ros::spinOnce();
        r.sleep();
    }

    qDebug("exit");

    return 0;
}
