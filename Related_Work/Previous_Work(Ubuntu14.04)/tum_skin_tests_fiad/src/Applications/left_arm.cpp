#include <tum_skin_bridge_fiad/PatchMarkers.h>
#include <QFileInfo>
#include <QDir>
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Point.h"

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32MultiArray.h"

#include <vector>

using namespace Tum::Skin::Fiad;
using namespace std;

/*** Number of cells in this part: ***/
int number_cells = 39;

int main( int argc, char** argv )
{
	ros::init(argc,argv,"left_arm");
	ros::NodeHandle nh;

	ros::Publisher id_pub = nh.advertise<std_msgs::Int32MultiArray>("ids_left_arm",1000);
	ros::Publisher range_pub = nh.advertise<std_msgs::Float32MultiArray>("range_left_arm",1000);
	ros::Publisher force_pub = nh.advertise<std_msgs::Float32MultiArray>("force_left_arm",1000);
	ros::Rate rate(250.0/4.0);	
	
	if (argc!=4)
	{
		ROS_ERROR_STREAM("wrong number of args:"<<argc);
		return -1;
	}

    QString arg=QString(argv[1]);
	bool ok;
    int patchId = arg.toUInt(&ok);

	if(!ok)
	{
        ROS_ERROR_STREAM("Invalid arg for patch id"<< arg.toStdString());
		return -1;
	}
	
    QString patchNamespace = QString(argv[2]);
    QString configFilePath = QString(argv[3]);
	if(!configFilePath.isEmpty())
	{
        QFileInfo fi(configFilePath);
		if(!fi.absoluteDir().exists())
		{
			ROS_ERROR_STREAM("Invalid file path"<<configFilePath.toStdString());
			return -1;		
		}	
	}
	QString tfParentName = QString(argv[4]);

    /*
     * create a Skin patch container
     */
	ROS_INFO_STREAM("Loading patch ...");
	PatchMarkers skinPatch1(patchId,tfParentName.toStdString(),patchNamespace.toStdString());

    /*** Get skin configuration: ***/
    skinPatch1.getSkinPatchConfiguration();

    Eigen::Matrix3d rot=Eigen::Matrix3d::Identity();
    Eigen::Vector3d pos=Eigen::Vector3d::Zero();
    skinPatch1.setSkinPatchTransformation(rot,pos);

    /*** Try to create objects with vector: ***/
    std::vector<tum_skin_msgs_fiad::SkinCell> cells(number_cells);

    std_msgs::Float32MultiArray range_msg; // define range_msg, it is a std_msgs with its type Float32MultiArray.
    std_msgs::Float32MultiArray force_msg;
    std_msgs::Int32MultiArray id_msg;

    /*** Define variables id, range, force for each cell with vector: ***/
    std::vector<int> ids(number_cells);
    std::vector<float> range(number_cells);
    std::vector<float> force(number_cells);

    while(ros::ok())
    {
        range_msg.data.clear();
        force_msg.data.clear();
        id_msg.data.clear();

        /*** Get data from each cell and transmit them into corresponding object (with a for-loop): ***/
        skinPatch1.publishPatchData();
        for (int i = 0; i < number_cells; i++)
        {
            skinPatch1.getSkinCellData(i,cells[i]);
        }


        /*** Get datas out of SkinCell message (with a for-loop): ***/
        for (int i = 0; i < number_cells; i++)
        {
            ids[i] = cells[i].module_id;
            range[i] = cells[i].proximity.at(0).range;
            force[i] = (cells[i].wrench.at(0).force.z+cells[i].wrench.at(1).force.z+cells[i].wrench.at(2).force.z)/3;
        }


        /*** Decide which cells are being touched (with a for-loop): ***/
        for (int i = 0; i < number_cells; i++)
        {
            if (range[i] >= 0.2)
                ROS_INFO_STREAM("Left arm: Cell " << i << " is being touched!");
        }


        /*** Put the data into corresponding rostopics: ***/
        for (int i = 0; i < number_cells; i++)
        {
            range_msg.data.push_back(range[i]);
            force_msg.data.push_back(force[i]);
            id_msg.data.push_back(ids[i]);
        }


        /*** Publish topics: ***/
        range_pub.publish(range_msg);
        force_pub.publish(force_msg);
        id_pub.publish(id_msg);

        ros::spinOnce();
        rate.sleep();
	}

	return 0;
}
