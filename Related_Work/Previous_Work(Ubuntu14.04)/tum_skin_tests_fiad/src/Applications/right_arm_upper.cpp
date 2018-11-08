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
	ros::init(argc,argv,"right_arm_upper");
	ros::NodeHandle nh;

	ros::Publisher id_pub = nh.advertise<std_msgs::Int32MultiArray>("ids_right_arm_upper",1000);
	ros::Publisher range_pub = nh.advertise<std_msgs::Float32MultiArray>("range_right_arm_upper",1000);
	ros::Publisher force_pub = nh.advertise<std_msgs::Float32MultiArray>("force_right_arm_upper",1000);
	ros::Rate rate(250.0/4.0);	
	
	if (argc!=4)
	{
		ROS_ERROR_STREAM("wrong number of args:"<<argc);
		return -1;
	}

	QString arg=QString(argv[1]);   //??????
	bool ok;
	int patchId = arg.toUInt(&ok); //??????

	if(!ok)
	{
		ROS_ERROR_STREAM("Invalid arg for patch id"<< arg.toStdString()); ///???
		return -1;
	}
	
	QString patchNamespace = QString(argv[2]); ///???
	QString configFilePath = QString(argv[3]); ///???
	if(!configFilePath.isEmpty())
	{
		QFileInfo fi(configFilePath); ///???
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
	// get skin config
    	skinPatch1.getSkinPatchConfiguration();

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
	tum_skin_msgs_fiad::SkinCell h;
	tum_skin_msgs_fiad::SkinCell i;
	tum_skin_msgs_fiad::SkinCell j;
	tum_skin_msgs_fiad::SkinCell k;
	tum_skin_msgs_fiad::SkinCell l;
	tum_skin_msgs_fiad::SkinCell m;
	tum_skin_msgs_fiad::SkinCell n;
	tum_skin_msgs_fiad::SkinCell o;
	tum_skin_msgs_fiad::SkinCell p;
	
	//int cnt=0;
	std_msgs::Float32MultiArray range_msg; // define rang_msg, it is a std_msgs with its type Float32MultiArray.
	std_msgs::Float32MultiArray force_msg;
	std_msgs::Int32MultiArray id_msg;
	
	int id_1,id_2,id_3,id_4,id_5,id_6,id_7,id_8,id_9,id_10,id_11,id_12,id_13,id_14,id_15,id_16;
	float range1,range2,range3,range4,range5,range6,range7,range8,range9,range10,range11,range12,range13,range14,range15,range16;
	float force1,force2,force3,force4,force5,force6,force7,force8,force9,force10,force11,force12,force13,force14,force15,force16;
	
	while(ros::ok())
	{
		range_msg.data.clear();	
		force_msg.data.clear();
		id_msg.data.clear();
		
		//publish viz marker
		skinPatch1.publishPatchData();
		skinPatch1.getSkinCellData(0,a);
		skinPatch1.getSkinCellData(1,b);
		skinPatch1.getSkinCellData(2,c);
		skinPatch1.getSkinCellData(3,d);
		skinPatch1.getSkinCellData(4,e);
		skinPatch1.getSkinCellData(5,f);
		skinPatch1.getSkinCellData(6,g);
		skinPatch1.getSkinCellData(7,h);
		skinPatch1.getSkinCellData(8,i);
		skinPatch1.getSkinCellData(9,j);
		skinPatch1.getSkinCellData(10,k);
		skinPatch1.getSkinCellData(11,l);
		skinPatch1.getSkinCellData(12,m);
		skinPatch1.getSkinCellData(13,n);
		skinPatch1.getSkinCellData(14,o);
		skinPatch1.getSkinCellData(15,p);

		//Get datas out of SkinCell message
		range1=	a.proximity.at(0).range;
		force1=(a.wrench.at(0).force.z+a.wrench.at(1).force.z+a.wrench.at(2).force.z)/3;	
		id_1=a.module_id;

		range2=	b.proximity.at(0).range;
		force2=(b.wrench.at(0).force.z+b.wrench.at(1).force.z+b.wrench.at(2).force.z)/3;	
		id_2=b.module_id;

		range3=	c.proximity.at(0).range;
		force3=(c.wrench.at(0).force.z+c.wrench.at(1).force.z+c.wrench.at(2).force.z)/3;	
		id_3=c.module_id;

		range4=	d.proximity.at(0).range;
		force4=(d.wrench.at(0).force.z+d.wrench.at(1).force.z+d.wrench.at(2).force.z)/3;	
		id_4=d.module_id;

		range5=	e.proximity.at(0).range;
		force5=(e.wrench.at(0).force.z+e.wrench.at(1).force.z+e.wrench.at(2).force.z)/3;	
		id_5=e.module_id;

		range6=	f.proximity.at(0).range;
		force6=(f.wrench.at(0).force.z+f.wrench.at(1).force.z+f.wrench.at(2).force.z)/3;	
		id_6=f.module_id;

		range7=	g.proximity.at(0).range;
		force7=(g.wrench.at(0).force.z+g.wrench.at(1).force.z+g.wrench.at(2).force.z)/3;	
		id_7=g.module_id;

		range8=	h.proximity.at(0).range;
		force8=(h.wrench.at(0).force.z+h.wrench.at(1).force.z+h.wrench.at(2).force.z)/3;	
		id_8=h.module_id;

		range9=	i.proximity.at(0).range;
		force9=(i.wrench.at(0).force.z+i.wrench.at(1).force.z+i.wrench.at(2).force.z)/3;	
		id_9=i.module_id;

		range10=j.proximity.at(0).range;
		force10=(j.wrench.at(0).force.z+j.wrench.at(1).force.z+j.wrench.at(2).force.z)/3;	
		id_10=j.module_id;

		range11=k.proximity.at(0).range;
		force11=(k.wrench.at(0).force.z+k.wrench.at(1).force.z+k.wrench.at(2).force.z)/3;	
		id_11=k.module_id;

		range12=l.proximity.at(0).range;
		force12=(l.wrench.at(0).force.z+l.wrench.at(1).force.z+l.wrench.at(2).force.z)/3;	
		id_12=l.module_id;

		range13=m.proximity.at(0).range;
		force13=(m.wrench.at(0).force.z+m.wrench.at(1).force.z+m.wrench.at(2).force.z)/3;	
		id_13=m.module_id;

		range14=n.proximity.at(0).range;
		force14=(n.wrench.at(0).force.z+n.wrench.at(1).force.z+n.wrench.at(2).force.z)/3;	
		id_14=n.module_id;

		range15=o.proximity.at(0).range;
		force15=(o.wrench.at(0).force.z+o.wrench.at(1).force.z+o.wrench.at(2).force.z)/3;	
		id_15=o.module_id;

		range16=p.proximity.at(0).range;
		force16=(p.wrench.at(0).force.z+p.wrench.at(1).force.z+p.wrench.at(2).force.z)/3;	
		id_16=p.module_id;


		if (range1>=0.2)
			{
				ROS_INFO_STREAM("Right cell_1 is being touched!!!"); 
			}
		if (range2>=0.2)
			{
				ROS_INFO_STREAM("Right cell_2 is being touched!!!"); 
			}
		if (range3>=0.2)
			{
				ROS_INFO_STREAM("Right cell_3 is being touched!!!"); 
			}
		if (range4>=0.2)
			{
				ROS_INFO_STREAM("Right cell_4 is being touched!!!"); 
			}
		if (range5>=0.2)
			{
				ROS_INFO_STREAM("Right cell_5 is being touched!!!"); 
			}
		if (range6>=0.2)
			{
				ROS_INFO_STREAM("Right cell_6 is being touched!!!"); 
			}
		if (range7>=0.2)
			{
				ROS_INFO_STREAM("Right cell_7 is being touched!!!"); 
			}
		if (range8>=0.2)
			{
				ROS_INFO_STREAM("Right cell_8 is being touched!!!"); 
			}
		if (range9>=0.2)
			{
				ROS_INFO_STREAM("Right cell_9 is being touched!!!"); 
			}
		if (range10>=0.2)
			{
				ROS_INFO_STREAM("Right cell_10 is being touched!!!"); 
			}
		if (range11>=0.2)
			{
				ROS_INFO_STREAM("Right cell_11 is being touched!!!"); 
			}
		if (range12>=0.2)
			{
				ROS_INFO_STREAM("Right cell_12 is being touched!!!"); 
			}
		if (range13>=0.2)
			{
				ROS_INFO_STREAM("Right cell_13 is being touched!!!"); 
			}
		if (range14>=0.2)
			{
				ROS_INFO_STREAM("Right cell_14 is being touched!!!"); 
			}
		if (range15>=0.2)
			{
				ROS_INFO_STREAM("Right cell_15 is being touched!!!"); 
			}
		if (range16>=0.2)
			{
				ROS_INFO_STREAM("Right cell_16 is being touched!!!"); 
			}


		//Put datas into topic msg
		range_msg.data.push_back(range1);
          	range_msg.data.push_back(range2);
        	range_msg.data.push_back(range3);
      	 	range_msg.data.push_back(range4);
      	  	range_msg.data.push_back(range5);
        	range_msg.data.push_back(range6);
      	 	range_msg.data.push_back(range7);
      	 	range_msg.data.push_back(range8);
      	 	range_msg.data.push_back(range9);
		range_msg.data.push_back(range10);
		range_msg.data.push_back(range11);
		range_msg.data.push_back(range12);
		range_msg.data.push_back(range13);
		range_msg.data.push_back(range14);
		range_msg.data.push_back(range15);
		range_msg.data.push_back(range16);

        	force_msg.data.push_back(force1);
        	force_msg.data.push_back(force2);
        	force_msg.data.push_back(force3);
        	force_msg.data.push_back(force4);
        	force_msg.data.push_back(force5);
        	force_msg.data.push_back(force6);
        	force_msg.data.push_back(force7);
        	force_msg.data.push_back(force8);
        	force_msg.data.push_back(force9);
        	force_msg.data.push_back(force10);
        	force_msg.data.push_back(force11);
        	force_msg.data.push_back(force12);
        	force_msg.data.push_back(force13);
        	force_msg.data.push_back(force14);
        	force_msg.data.push_back(force15);
        	force_msg.data.push_back(force16);

        	id_msg.data.push_back(id_1);
        	id_msg.data.push_back(id_2);
        	id_msg.data.push_back(id_3);
        	id_msg.data.push_back(id_4);
        	id_msg.data.push_back(id_5);
        	id_msg.data.push_back(id_6);
        	id_msg.data.push_back(id_7);
		id_msg.data.push_back(id_8);
		id_msg.data.push_back(id_9);
        	id_msg.data.push_back(id_10);
        	id_msg.data.push_back(id_11);
		id_msg.data.push_back(id_12);
		id_msg.data.push_back(id_13);
        	id_msg.data.push_back(id_14);
		id_msg.data.push_back(id_15);
		id_msg.data.push_back(id_16);
        	

		//publish topics
		range_pub.publish(range_msg);
        	force_pub.publish(force_msg);
       		id_pub.publish(id_msg);

        	//cnt++;
        	ros::spinOnce();
        	rate.sleep();
	}
	return 0;

}
