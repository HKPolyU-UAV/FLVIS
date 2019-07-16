#ifndef TIC_TOC_H
#define TIC_TOC_H

#include "ros/time.h"


//usage:
//tic_toc_ros tt;
//tt.dT_s(); will return the dT in second;
//tt.dT_ms(); will return the dT in mili-second


class tic_toc_ros
{
public:
    tic_toc_ros(void) {
        tic=ros::Time::now();
    }
    double dT_s(void){
        return (ros::Time::now()-tic).toSec();
    }
    double dT_ms(void){
        return (ros::Time::now()-tic).toSec()*1000;
    }
    void toc(void)
    {
        std::cout << (ros::Time::now()-tic).toSec()*1000 <<"ms" << std::endl;
    }
    void toc(std::string str)
    {
        std::cout << str << " time:" <<(ros::Time::now()-tic).toSec()*1000 <<"ms" << std::endl;
    }
private:
    ros::Time tic;
//    ros::Time toc;
};



#endif // TIC_TOC_H
