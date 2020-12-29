#ifndef CVDRAW_H
#define CVDRAW_H

#include <include/common.h>
#include <include/camera_frame.h>


inline void drawFPS(cv::Mat& img, int fps)
{
    putText(img, "FPS:"+std::to_string(fps), cv::Point(0,20), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,255,0), 2, cv::LINE_AA);
}

inline void drawRegion16(cv::Mat& img)
{
    int divH = floor(img.size().height/4);
    int divW = floor(img.size().width/4);
    int x,y;
    for(int i=1; i<=3; i++)//horizon
    {
        y=i*divH;
        cv::line(img, cv::Point(0,y), cv::Point((img.size().width-1),y), cv::Scalar(255,255,255),2);//horizon
        x=i*divW;
        cv::line(img, cv::Point(x,0), cv::Point(x,(img.size().height-1)), cv::Scalar(255,255,255),2);//vertical
    }
}

inline void drawKeyPts(cv::Mat& img, const vector<cv::Point2f>& KeyPts)
{
    for(size_t i=0; i<KeyPts.size(); i++)
    {
        cv::Point pt(floor(KeyPts.at(i).x),floor(KeyPts.at(i).y));
        cv::circle(img, pt, 2, cv::Scalar( 255, 0, 0 ), 3);
    }
}

inline void drawOutlier(cv::Mat& img, const vector<Vec2>& outlier)
{
    for(size_t i=0; i<outlier.size(); i++)
    {
        cv::Point pt(floor(outlier.at(i)[0]),floor(outlier.at(i)[1]));
        cv::circle(img, pt, 2, cv::Scalar( 255, 255, 255 ), 2);
    }
}

inline void drawFlow(cv::Mat& img, const vector<cv::Point2f>& from, const vector<cv::Point2f>& to)
{
    if(from.size()==to.size())
    {
        for(size_t i=0; i<from.size(); i++)
        {
            cv::circle(img, from.at(i), 1, cv::Scalar( 0, 255, 0 ), 1);
            cv::line  (img, from.at(i), to.at(i), cv::Scalar( 0,204,204),1);
        }
    }
}

inline void drawFrame(cv::Mat& img, CameraFrame& frame, int min, int max)
{
    drawRegion16(img);
    int fps=floor(1.0/frame.solving_time);
    if(fps>0&&fps<500)
    {
        cv::putText(img, "FPS:"+std::to_string(fps),
                    cv::Point(0,20), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,255,0), 2, cv::LINE_8);
    }
    std::stringstream stream;
    stream << std::fixed << std::setprecision(2) << frame.reprojection_error;
    cv::putText(img, "ERR:"+stream.str(),
                cv::Point(img.cols-150,20), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,255,0), 2, cv::LINE_8);
    int gap= floor(250/(max-min));
    for(auto lm:frame.landmarks)
    {
        if(lm.hasDepthInf())
        {
            float z=lm.lm_3d_c[2];
            if(z>=max) z=max;
            if(z<min)  z=min;
            int b=floor((z-min)*gap);
            int r=255-b;
            cv::Point pt(round(lm.lm_2d_plane[0]),round(lm.lm_2d_plane[1]));
            cv::circle(img, pt, 3, cv::Scalar( b, 0, r ), 3);
//            if(lm.is_belong_to_kf)
//            {
//                cv::circle(img, pt, 3, cv::Scalar( b, 0, r ), 4);
//            }
//            else
//            {
//                cv::circle(img, pt, 2, cv::Scalar( b, 0, r ), 2);
//            }
        }
    }
}

inline void visualizeDepthImg(cv::Mat& visualized_depth, CameraFrame& frame)
{
    cv::Mat d_img=frame.d_img;
    int size=d_img.cols*d_img.rows;
    for(int i=0; i<size; i++)
    {
        if(isnan(d_img.at<ushort>(i)))
        {
            d_img.at<ushort>(i)=0;
        }
        if(d_img.at<ushort>(i)>10000||d_img.at<ushort>(i)<200)
        {
            d_img.at<ushort>(i)=0;
        }
    }
    cv::Mat adjMap;
    d_img.convertTo(adjMap,CV_8UC1, 255 / (10000.0), 0);
    cv::applyColorMap(adjMap, visualized_depth, cv::COLORMAP_RAINBOW);
    for(int i=0; i<size; i++)
    {
        if(d_img.at<ushort>(i)==0)
        {
            cv::Vec3b color = visualized_depth.at<cv::Vec3b>(i);
            color[0]=255;
            color[1]=255;
            color[2]=255;
            visualized_depth.at<cv::Vec3b>(i)=color;
        }
    }
}

#endif // CVDRAW_H
