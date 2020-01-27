#include "include/f2f_tracking.h"

F2FTracking::F2FTracking(int width_in,int height_in)
{
    this->width=width_in;
    this->height=height_in;
}


bool F2FTracking::tracking(CameraFrame& from,
                           CameraFrame& to,
                           vector<Vec2>& lm2d_from,
                           vector<Vec2>& lm2d_to,
                           vector<Vec2>& outlier)
{
    //STEP1: Optical Flow
    int outlier_untracked_cnt=0;
    int outlier_orb_unmatch_cnt=0;

    vector<cv::Point2f> from_cvP2f = from.get2dPtsVec_cvP2f();
    vector<cv::Point2f> tracked_cvP2f;
    vector<cv::Mat>     trackedLMDescriptors;
    vector<float>   err;
    vector<unsigned char> mask_tracked;
    vector<unsigned char> mask_hasorb;
    vector<unsigned char> mask_matched;
    cv::TermCriteria criteria = cv::TermCriteria((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS), 10, 0.03);

    cv::calcOpticalFlowPyrLK(from.img, to.img, from_cvP2f, tracked_cvP2f,
                         mask_tracked, err, cv::Size(20,20), 5, criteria);


    cv::Ptr<cv::DescriptorExtractor> extractor = cv::ORB::create();
    if(0)//use patch
    {
        for(size_t i=0; i<tracked_cvP2f.size(); i++)
        {
            if(mask_tracked.at(i)==0
                    ||tracked_cvP2f.at(i).x<2
                    ||tracked_cvP2f.at(i).y<2
                    ||tracked_cvP2f.at(i).x>(width-3)
                    ||tracked_cvP2f.at(i).y>(height-3))
            {//For every valid tracking
                //outlier_untracked_cnt++;
                mask_tracked.at(i)=0;
                mask_hasorb.push_back(0);
                cv::Mat zeroDescriptor(cv::Size(32, 1), CV_8U, cv::Scalar(0));
                trackedLMDescriptors.push_back(zeroDescriptor);
                continue;
            }
            //Find cloest point in 5x5 Region
            vector<cv::Point2f>      tracked_lm_and_patch_cvP2f;
            vector<cv::KeyPoint>     tracked_lm_and_patch_cvKP;
            tracked_lm_and_patch_cvP2f.clear();
            tracked_lm_and_patch_cvKP.clear();
            // 9,10,11,12,13
            //24, 1, 2, 3,14
            //23, 8, 0, 4,15
            //22, 7, 6, 5,16
            //21,20,19,18,17
            //25 refer to tracked point 0 refer to the center;
            int x=round(tracked_cvP2f.at(i).x);
            int y=round(tracked_cvP2f.at(i).y);
            //Patch33
            tracked_lm_and_patch_cvP2f.push_back(cv::Point2f(x,y));    //0
            tracked_lm_and_patch_cvP2f.push_back(cv::Point2f(x-1,y-1));//1
            tracked_lm_and_patch_cvP2f.push_back(cv::Point2f(x,  y-1));//2
            tracked_lm_and_patch_cvP2f.push_back(cv::Point2f(x+1,y-1));//3
            tracked_lm_and_patch_cvP2f.push_back(cv::Point2f(x+1,y));  //4
            tracked_lm_and_patch_cvP2f.push_back(cv::Point2f(x+1,y+1));//5
            tracked_lm_and_patch_cvP2f.push_back(cv::Point2f(x  ,y+1));//6
            tracked_lm_and_patch_cvP2f.push_back(cv::Point2f(x-1,y+1));//7
            tracked_lm_and_patch_cvP2f.push_back(cv::Point2f(x-1,y  ));//8
            //        //Patch55
            //        tracked_lm_and_patch_cvP2f.push_back(Point2f(x-2,y-2));//9
            //        tracked_lm_and_patch_cvP2f.push_back(Point2f(x-1,y-2));//10
            //        tracked_lm_and_patch_cvP2f.push_back(Point2f(x  ,y-2));//11
            //        tracked_lm_and_patch_cvP2f.push_back(Point2f(x+1,y-2));//12
            //        tracked_lm_and_patch_cvP2f.push_back(Point2f(x+2,y-2));//13
            //        tracked_lm_and_patch_cvP2f.push_back(Point2f(x+2,y-1));//14
            //        tracked_lm_and_patch_cvP2f.push_back(Point2f(x+2,y  ));//15
            //        tracked_lm_and_patch_cvP2f.push_back(Point2f(x+2,y+1));//16
            //        tracked_lm_and_patch_cvP2f.push_back(Point2f(x+2,y+2));//17
            //        tracked_lm_and_patch_cvP2f.push_back(Point2f(x+1,y+2));//18
            //        tracked_lm_and_patch_cvP2f.push_back(Point2f(x  ,y+2));//19
            //        tracked_lm_and_patch_cvP2f.push_back(Point2f(x-1,y+2));//20
            //        tracked_lm_and_patch_cvP2f.push_back(Point2f(x-2,y+2));//21
            //        tracked_lm_and_patch_cvP2f.push_back(Point2f(x-2,y+1));//22
            //        tracked_lm_and_patch_cvP2f.push_back(Point2f(x-2,y  ));//23
            //        tracked_lm_and_patch_cvP2f.push_back(Point2f(x-2,y-1));//24
            tracked_lm_and_patch_cvP2f.push_back(tracked_cvP2f.at(i));
            cv::Mat descriptorsMat;
            cv::KeyPoint::convert(tracked_lm_and_patch_cvP2f,tracked_lm_and_patch_cvKP);
            extractor->compute(to.img, tracked_lm_and_patch_cvKP, descriptorsMat);
            if(tracked_lm_and_patch_cvKP.size()>0)
            {
                mask_hasorb.push_back(1);
            }else {
                mask_hasorb.push_back(0);
                cv::Mat zeroDescriptor(cv::Size(32, 1), CV_8U, cv::Scalar(0));
                trackedLMDescriptors.push_back(zeroDescriptor);
                continue;
            }
            int cloest_idx=0;
            int min_distance=999;
            for(int patch_idx=0;patch_idx<tracked_lm_and_patch_cvKP.size();patch_idx++)
            {
                double dis=norm(from.landmarks.at(i).lm_descriptor, descriptorsMat.row(patch_idx), cv::NORM_HAMMING);
                if( dis <= min_distance)
                {
                    min_distance=dis;
                    cloest_idx=patch_idx;
                }
            }
            tracked_cvP2f.at(i).x=tracked_lm_and_patch_cvP2f.at(cloest_idx).x;
            tracked_cvP2f.at(i).y=tracked_lm_and_patch_cvP2f.at(cloest_idx).y;
            trackedLMDescriptors.push_back(descriptorsMat.row(cloest_idx));
        }

    }else {//don't use patch
        vector<cv::KeyPoint>     tracked_lm_cvKP;
        cv::Mat descriptorsMat;

        cv::KeyPoint::convert(tracked_cvP2f,tracked_lm_cvKP);
        extractor->compute(to.img, tracked_lm_cvKP, descriptorsMat);

        for(size_t i=0; i<tracked_cvP2f.size(); i++)
        {
            unsigned char hasDescriptor = 0;
            for(size_t j=0; j<tracked_lm_cvKP.size(); j++)
            {
                if(tracked_cvP2f.at(i).x==tracked_lm_cvKP.at(j).pt.x &&
                        tracked_cvP2f.at(i).y==tracked_lm_cvKP.at(j).pt.y)
                {//has ORB descriptor
                    trackedLMDescriptors.push_back(descriptorsMat.row(j));
                    hasDescriptor = 1;
                    break;
                }
            }
            mask_hasorb.push_back(hasDescriptor);
            if(hasDescriptor==0)
            {
                cv::Mat zeroDescriptor(cv::Size(32, 1), CV_8U, cv::Scalar(0));
                trackedLMDescriptors.push_back(zeroDescriptor);
            }
        }
    }

    for(size_t i=0; i<trackedLMDescriptors.size(); i++)
    {
        if(norm(from.landmarks.at(i).lm_descriptor, trackedLMDescriptors.at(i), cv::NORM_HAMMING) <= 30)
        {
            //landmarks.at(i).lm_descriptor = trackedLMDescriptors.at(i);
            mask_matched.push_back(1);
        }else
        {
            outlier_orb_unmatch_cnt++;
            mask_matched.push_back(0);
        }
    }

    //copy to new frame
    bool reuslt = false;
    for(int i=from.landmarks.size()-1; i>=0; i--)
    {
        if(mask_tracked.at(i)!=1      ||
                mask_hasorb.at(i)!=1  ||
                mask_matched.at(i)!=1)
        {//outliers
            outlier.push_back(from.landmarks.at(i).lm_2d);
            //tracked_cvP2f.erase(tracked_cvP2f.begin()+i);
            //from.landmarks.erase(from.landmarks.begin()+i);
        }else
        {//inliers
            lm2d_from.push_back(from.landmarks.at(i).lm_2d);
            lm2d_to.push_back(Vec2(tracked_cvP2f.at(i).x,tracked_cvP2f.at(i).y));
            LandMarkInFrame lm=from.landmarks.at(i);
            lm.lm_2d=Vec2(tracked_cvP2f.at(i).x,tracked_cvP2f.at(i).y);
            to.landmarks.push_back(lm);
        }
    }

    if(lm2d_to.size()>=20 && outlier.size()>lm2d_to.size())
    {
        reuslt=true;
    }

    return reuslt;
}
