#include <straight_fsm/query_variable.h>
#include<math.h>
using namespace std;
queryVariable::queryVariable()
{

    sub1 = nh.subscribe("VechInfo", 1, &queryVariable::callback1, this);
    sub2 = nh.subscribe("finalObjList/map/objects", 1, &queryVariable::callback2, this);
    sub3 = nh.subscribe("lidar_objlist", 1, &queryVariable::callback3, this);

}



void queryVariable::callback1(const vaafo_msgs::CarInfo msg){

    egoCar = msg;
}

void queryVariable::callback2(const vaafo_msgs::DetectedObjectArray msg){

    objects = msg.objects;
}

void queryVariable::callback3(const vaafo_msgs::SensorObjectList msg){
    lidar_objects = msg.objects;
}




/*
 * get the relative postions and velocities of the objects around our ego car through the lidar_objectlist
 * reassign those results to the array otherObjects
 * otherObjects[0] = front car;
 * otherObjects[1] = front left lane car;
 * otherObjects[2] = front right lane car;
 * otherObjects[3] = behind left lane car;
 * otherObjects[4] = behind right lane car;
 * */


void queryVariable::get_otherObjects(){
     //vaafo_msgs::DetectedObject nearObjects[]={f_Object,fl_Object,fb_Object,fr_Object,br_Object};

    int num = lidar_objects.size();
    int index[5]={0};

    if(num!=0){
        for (int i=0;i<num;i++){
            double s = lidar_objects.at(i).pose.position.x;
            double d = lidar_objects.at(i).pose.position.y;

            if (d<=1.8 && d>=-1.8 && s>0){
                index[0]=1;
                otherObjects[0]=lidar_objects.at(i);
            }
            if(d>3){
                // left front car
                if(s>0){
                    index[1]=2;
                    otherObjects[1] = lidar_objects.at(i);}
                else{
                    // left back car
                    index[2]=3;
                    otherObjects[2] = lidar_objects.at(i);}
            }
            else if(d<-3){
                if(s>0){
                    // front right car
                    index[3]=4;
                    otherObjects[3] = lidar_objects.at(i);
                }
                else {
                    // back right car
                    index[4]=5;
                    otherObjects[4] = lidar_objects.at(i);
                }
            }

        }
    }
    // reassign the value to no sensor objects
    for (int i=0;i<5;i++){
        if (index[i]==0){
            if(i==0 || i==1 ||i==3){
                otherObjects[i].pose.position.x = 1500;
                otherObjects[i].pose.position.y = 1500;
                otherObjects[i].velocity.linear.x = 999;
                otherObjects[i].velocity.linear.y = 999;
            }
            else{
                otherObjects[i].pose.position.x = -1500;
                otherObjects[i].pose.position.y = -1500;
                otherObjects[i].velocity.linear.x = -999;
                otherObjects[i].velocity.linear.y = -999;
            }

        }

    }
    cout<<"other objects are:"<<endl;
    for (int i=0;i<5;i++){
        cout<<"the object "<<i<<" is:"<<otherObjects[i].pose.position.x<<endl;
}

    }



double* queryVariable::computeD(vaafo_msgs::DetectedObject object){
    // test result: if the object runs on the right lane, d=+3.5;
    // if the object runs on the left side, d=-3.5
    double x0 = egoCar.pose.position.x;
    double y0 = egoCar.pose.position.y;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(egoCar.pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    //double theta = pi-yaw;
    double theta = yaw;

    double x2 = object.pose.position.x;
    double y2 = object.pose.position.y;
    double c = y0-tan(theta)*x0;
    double d = (tan(theta)*x2-y2+c)/sqrt(pow(tan(theta), 2)+1);
    double cosalpha = (x2-x0+(y2-y0)*tan(theta)) /
            (sqrt(pow(x2-x0,2)+pow(y2-y0,2)) * sqrt(1+pow(tan(theta), 2)));
    double* result = new double[2];

    result[0] = d;
    result[1] = cosalpha;
    return result;
}


void queryVariable::coord_frenet(){


    double x0 = egoCar.pose.position.x;
    double y0 = egoCar.pose.position.y;
    tf::Quaternion quat_0;
    tf::quaternionMsgToTF(egoCar.pose.orientation, quat_0);
    double roll_0, pitch_0, yaw_0;
    tf::Matrix3x3(quat_0).getRPY(roll_0, pitch_0, yaw_0);
    double theta = yaw_0;
    double alpha = atan(y0/x0);

    std::cout<<"the yaw of ego car is:"<<yaw_0<<std::endl;
    std::cout<<alpha<<std::endl;
    double new_x = x0*cos(alpha)+y0*sin(alpha);
    double new_y = -x0*sin(alpha)+ y0*cos(alpha);



    std::cout<<"the x pos of ego car is:"<<new_x<<std::endl;
    std::cout<<"the old x pos of ego car is:"<<x0<<std::endl;
    std::cout<<"the y pos of ego car is:"<<new_y<<std::endl;
    std::cout<<"the old y pos of ego car is:"<<y0<<std::endl;


    int num =objects.size();
    if (num !=0){
        for (int var = 0; var < num; ++var) {
            tf::Quaternion quat_1;
            tf::quaternionMsgToTF(objects.at(var).pose.orientation, quat_1);
            double roll_1, pitch_1, yaw_1;
            tf::Matrix3x3(quat_1).getRPY(roll_1, pitch_1, yaw_1);

        }

    }

    int num2 =lidar_objects.size();
    if (num2 !=0){
        for (int var = 0; var < num2; ++var) {
            tf::Quaternion quat_2;
            tf::quaternionMsgToTF(lidar_objects.at(var).pose.orientation, quat_2);
            double roll_2, pitch_2, yaw_2;
            tf::Matrix3x3(quat_2).getRPY(roll_2, pitch_2, yaw_2);

        }

    }


}


