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


using namespace Tum::Skin::Fiad;

int main( int argc, char** argv )
{
    ros::init(argc, argv, "viz_test");

    ros::NodeHandle n;

    ros::Publisher range_pub = n.advertise<std_msgs::Float32MultiArray>("range_topic",1000);
    ros::Publisher force_pub = n.advertise<std_msgs::Float32MultiArray>("force_topic",1000);
    ros::Publisher accx_pub = n.advertise<std_msgs::Float32MultiArray>("accx_topic",1000);
    ros::Publisher accy_pub = n.advertise<std_msgs::Float32MultiArray>("accy_topic",1000);
    ros::Publisher accz_pub = n.advertise<std_msgs::Float32MultiArray>("accz_topic",1000);
    ros::Publisher id_pub = n.advertise<std_msgs::Int32MultiArray>("/ids",1000);

    ros::Rate r(250.0/4.0);


    ROS_INFO_STREAM("argc=" << argc );
    ROS_INFO_STREAM("argv[0]=" << argv[0] );
    ROS_INFO_STREAM("argv[1]=" << argv[1] );
    ROS_INFO_STREAM("argv[2]=" << argv[2] );
    ROS_INFO_STREAM("argv[3]=" << argv[3] );
    ROS_INFO_STREAM("argv[4]=" << argv[4] );

    if(argc != 4)
    {
        ROS_ERROR_STREAM("Wrong number of args: " << argc);
       return -1;
    }

    QString arg = QString(argv[1]);
    bool ok;

    int patchId = arg.toUInt(&ok);
    if(!ok)
    {
        ROS_ERROR_STREAM("Invalid arg for patch id " << arg.toStdString());
        return -1;
    }

    QString patchNamespace = QString(argv[2]);

    QString configFilePath = QString(argv[3]);
    if(!configFilePath.isEmpty())
    {
        QFileInfo fi(configFilePath);
        if(!fi.absoluteDir().exists())
        {
            ROS_ERROR_STREAM("Invalid file path " << configFilePath.toStdString());
            return -1;
        }
    }


    /*
     * create a Skin patch container
     */

    ROS_INFO_STREAM("Loading patch ...");
    PatchMarkers skinPatch1(patchId,"/world",patchNamespace.toStdString());

    // get skin config
    skinPatch1.getSkinPatchConfiguration();
    //    skinPatch1.getSkinPatchConfiguration("/home/flo/Desktop/SkinConfs/SkinConfigs/patch1.ini");
    //    skinPatch1.getSkinPatchConfiguration("/home/flo/Desktop/SkinConfs/DistributedSkinConfigs/patch1_new11.ini");

    //    skinPatch1.setLEDColorG(1,1);

    Eigen::Matrix3d rot=Eigen::Matrix3d::Identity();
    Eigen::Vector3d pos=Eigen::Vector3d::Zero();

    skinPatch1.setSkinPatchTransformation(rot,pos);

    tum_skin_msgs_fiad::SkinCell a;
    tum_skin_msgs_fiad::SkinCell b;
    tum_skin_msgs_fiad::SkinCell c;
    tum_skin_msgs_fiad::SkinCell d;
    tum_skin_msgs_fiad::SkinCell e;
    tum_skin_msgs_fiad::SkinCell f;
    tum_skin_msgs_fiad::SkinCell g;

    int cnt=0;
    std_msgs::Float32MultiArray range_msg;

    std_msgs::Float32MultiArray force_msg;

    std_msgs::Float32MultiArray accx_msg;  

    std_msgs::Float32MultiArray accy_msg;  

    std_msgs::Float32MultiArray accz_msg;  

    std_msgs::Int32MultiArray id_msgs;

    int id_a, id_b, id_c, id_d, id_e, id_f,id_g;

    float range1, range2, range3, range4, range5, range6, range7;

    float force1, force2, force3, force4, force5, force6, force7;

    float accx1, accx2, accx3, accx4, accx5, accx6, accx7;

    float accy1, accy2, accy3, accy4, accy5, accy6, accy7;

    float accz1, accz2, accz3, accz4, accz5, accz6, accz7;

    while(ros::ok())
    {

        range_msg.data.clear();
        force_msg.data.clear();
        accx_msg.data.clear();
        accy_msg.data.clear();
        accz_msg.data.clear();
        id_msgs.data.clear();


        // publish viz marker
        skinPatch1.publishPatchData();
        skinPatch1.getSkinCellData(0,a);
        skinPatch1.getSkinCellData(1,b);
        skinPatch1.getSkinCellData(2,c);
        skinPatch1.getSkinCellData(3,d);
        skinPatch1.getSkinCellData(4,e);
        skinPatch1.getSkinCellData(5,f);
        skinPatch1.getSkinCellData(6,g);

        //Get datas out of SkinCell message
        range1 = a.proximity.at(0).range;
        force1 = (a.wrench.at(0).force.z + a.wrench.at(1).force.z + a.wrench.at(2).force.z)/3;
        accx1 = a.acc.linear_acceleration.x;
        accy1 = a.acc.linear_acceleration.y;
        accz1 = a.acc.linear_acceleration.z;
        id_a = a.module_id;

        range2 = b.proximity.at(0).range;
        force2 = (b.wrench.at(0).force.z + b.wrench.at(1).force.z + b.wrench.at(2).force.z)/3;
        accx2 = b.acc.linear_acceleration.x;
        accy2 = b.acc.linear_acceleration.y;
        accz2 = b.acc.linear_acceleration.z;
        id_b = b.module_id;

        range3 = c.proximity.at(0).range;
        force3 = (c.wrench.at(0).force.z + c.wrench.at(1).force.z + c.wrench.at(2).force.z)/3;
        accx3 = c.acc.linear_acceleration.x;
        accy3 = c.acc.linear_acceleration.y;
        accz3 = c.acc.linear_acceleration.z;
        id_c = c.module_id;

        range4 = d.proximity.at(0).range;
        force4 = (d.wrench.at(0).force.z + d.wrench.at(1).force.z + d.wrench.at(2).force.z)/3;
        accx4 = d.acc.linear_acceleration.x;
        accy4 = d.acc.linear_acceleration.y;
        accz4 = d.acc.linear_acceleration.z;
        id_d = d.module_id;

        range5 = e.proximity.at(0).range;
        force5 = (e.wrench.at(0).force.z + e.wrench.at(1).force.z + e.wrench.at(2).force.z)/3;
        accx5 = e.acc.linear_acceleration.x;
        accy5 = e.acc.linear_acceleration.y;
        accz5 = e.acc.linear_acceleration.z;
        id_e = e.module_id;

        range6 = f.proximity.at(0).range;
        force6 = (f.wrench.at(0).force.z + f.wrench.at(1).force.z + f.wrench.at(2).force.z)/3;
        accx6 = f.acc.linear_acceleration.x;
        accy6 = f.acc.linear_acceleration.y;
        accz6 = f.acc.linear_acceleration.z;
        id_f = f.module_id;

        range7 = g.proximity.at(0).range;
        force7 = (g.wrench.at(0).force.z + g.wrench.at(1).force.z + g.wrench.at(2).force.z)/3;
        accx7 = g.acc.linear_acceleration.x;
        accy7 = g.acc.linear_acceleration.y;
        accz7 = g.acc.linear_acceleration.z;
        id_g = g.module_id;

        //Put datas into topic msg
        range_msg.data.push_back(range1);
        range_msg.data.push_back(range2);
        range_msg.data.push_back(range3);
        range_msg.data.push_back(range4);
        range_msg.data.push_back(range5);
        range_msg.data.push_back(range6);
        range_msg.data.push_back(range7);

        force_msg.data.push_back(force1);
        force_msg.data.push_back(force2);
        force_msg.data.push_back(force3);
        force_msg.data.push_back(force4);
        force_msg.data.push_back(force5);
        force_msg.data.push_back(force6);
        force_msg.data.push_back(force7);

        accx_msg.data.push_back(accx1);
        accx_msg.data.push_back(accx2);
        accx_msg.data.push_back(accx3);
        accx_msg.data.push_back(accx4);
        accx_msg.data.push_back(accx5);
        accx_msg.data.push_back(accx6);
        accx_msg.data.push_back(accx7);

        accy_msg.data.push_back(accy1);
        accy_msg.data.push_back(accy2);
        accy_msg.data.push_back(accy3);
        accy_msg.data.push_back(accy4);
        accy_msg.data.push_back(accy5);
        accy_msg.data.push_back(accy6);
        accy_msg.data.push_back(accy7);

        accz_msg.data.push_back(accz1);
        accz_msg.data.push_back(accz2);
        accz_msg.data.push_back(accz3);
        accz_msg.data.push_back(accz4);
        accz_msg.data.push_back(accz5);
        accz_msg.data.push_back(accz6);
        accz_msg.data.push_back(accz7);

        id_msgs.data.push_back(id_a);
        id_msgs.data.push_back(id_b);
        id_msgs.data.push_back(id_c);
        id_msgs.data.push_back(id_d);
        id_msgs.data.push_back(id_e);
        id_msgs.data.push_back(id_f);
        id_msgs.data.push_back(id_g);

        //Publish topics
        range_pub.publish(range_msg);
        force_pub.publish(force_msg);
        accx_pub.publish(accx_msg);
        accy_pub.publish(accy_msg);
        accz_pub.publish(accz_msg);

        id_pub.publish(id_msgs);

        cnt++;
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}




