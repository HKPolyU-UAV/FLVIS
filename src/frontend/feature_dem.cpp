#include <include/feature_dem.h>
#include <opencv2/features2d/features2d.hpp>

// Driver function to sort the vector elements by
// second element of pair in descending order
static bool sortbysecdesc(const pair<cv::Point2f,float> &a,
                          const pair<cv::Point2f,float> &b)
{
    return a.second>b.second;
}

FeatureDEM::FeatureDEM(const int image_width,
                       const int image_height,
                       const Vec6 f_para)
{
    width=image_width;
    height=image_height;
    regionWidth  = floor(width/4.0);
    regionHeight = floor(height/4.0);
    boundary_dis = floor(f_para(2)/2.0);
    max_region_feature_num = f_para(0);
    min_region_feature_num = f_para(1);
    gftt_num = (int)f_para(3);
    gftt_ql = (double)f_para(4);
    gftt_dis = (int)f_para(5);
    cout << "max_region_feature_num:" << max_region_feature_num << endl;
    cout << "min_region_feature_num:" << min_region_feature_num << endl;
    cout << "boundary_dis:" << boundary_dis << endl;
    cout << "gftt_num:" << gftt_num << endl;
    cout << "gftt_ql:" << gftt_ql << endl;
    cout << "gftt_dis:" << gftt_dis << endl;
    int gridx[5],gridy[5];
    for(int i=0; i<5; i++)
    {
        gridx[i]=i*regionWidth;
        gridy[i]=i*regionHeight;
    }
    for(int i=0; i<16; i++)
    {
        detectorMask[i] = cv::Mat(image_height, image_width, CV_8S,cv::Scalar(0));
        int x_begin,x_end,y_begin,y_end;
        x_begin = gridx[i%4];
        x_end   = gridx[(i%4)+1];
        y_begin = gridy[i/4];
        y_end   = gridy[(i/4)+1];
        for(int xx=x_begin; xx<x_end; xx++)
        {
            for(int yy=y_begin; yy<y_end; yy++)
            {
                detectorMask[i].at<schar>(yy,xx)=1;
            }
        }
    }
}

FeatureDEM::~FeatureDEM()
{;}

void FeatureDEM::calHarrisR(const cv::Mat& img,
                            cv::Point2f& Pt,
                            float &R)
{
    uchar patch[9];
    int xx = Pt.x;
    int yy = Pt.y;
    patch[0]=img.at<uchar>(cv::Point(xx-1,yy-1));
    patch[1]=img.at<uchar>(cv::Point(xx,yy-1));
    patch[2]=img.at<uchar>(cv::Point(xx+1,yy-1));
    patch[3]=img.at<uchar>(cv::Point(xx-1,yy));
    patch[4]=img.at<uchar>(cv::Point(xx,yy));
    patch[5]=img.at<uchar>(cv::Point(xx+1,yy+1));
    patch[6]=img.at<uchar>(cv::Point(xx-1,yy+1));
    patch[7]=img.at<uchar>(cv::Point(xx,yy+1));
    patch[8]=img.at<uchar>(cv::Point(xx+1,yy+1));
    float IX,IY;
    float X2,Y2;
    float XY;
    IX = (patch[0]+patch[3]+patch[6]-(patch[2]+patch[5]+patch[8]))/3;
    IY = (patch[0]+patch[1]+patch[2]-(patch[6]+patch[7]+patch[8]))/3;
    X2 = IX*IX;
    Y2 = IY*IX;
    XY = IX*IX;
    //M = | X2  XY |
    //    | XY  Y2 |
    //R = det(M)-k(trace^2(M))
    //  = X2*Y2-XY*XY  - 0.05*(X2+Y2)*(X2+Y2)
    R = (X2*Y2)-(XY*XY) - 0.05*(X2+Y2)*(X2+Y2);
}


//Devided all features into 16 regions
void FeatureDEM::fillIntoRegion(const cv::Mat& img, const vector<cv::Point2f>& pts,
                                vector<pair<cv::Point2f,float>> (&region)[16], bool existed_features)
{
    if(existed_features)
    {//do not calculate harris value
        for(size_t i=0; i<pts.size(); i++)
        {
            cv::Point2f pt = pts.at(i);
            int regionNum= 4*floor(pt.y/regionHeight) + (pt.x/regionWidth);
            region[regionNum].push_back(make_pair(pt,99999.0));
        }
    }
    else
    {
        for(size_t i=0; i<pts.size(); i++)
        {
            cv::Point2f pt = pts.at(i);
            if (pt.x>=10 && pt.x<(width-10) && pt.y>=10 && pt.y<(height-10))
            {
                float Harris_R;
                calHarrisR(img,pt,Harris_R);
                int regionNum= 4*floor(pt.y/regionHeight) + (pt.x/regionWidth);
                region[regionNum].push_back(make_pair(pt,Harris_R));
            }
        }
    }
}


void FeatureDEM::redetect(const cv::Mat& img,
                          const vector<Vec2>& existedPts,
                          vector<Vec2>& newPts,
                          int &newPtscount)
{
    //Clear
    newPts.clear();
    newPtscount=0;
    vector<cv::Point2f> new_features;
    new_features.clear();

    //fill the existed features into region
    vector<cv::Point2f> existedPts_cvP2f=vVec2_2_vcvP2f(existedPts);
    for(int i=0; i<16; i++)
    {
        regionKeyPts[i].clear();
    }
    fillIntoRegion(img,existedPts_cvP2f,regionKeyPts,true);

    //
    cv::Mat mask = cv::Mat(height, width, CV_8UC1, cv::Scalar(255));
    //over exposure mask
    int over_exp_block_cnt=0;
    for(int xx=10; xx < img.cols-10; xx+=10)
    {
        for(int yy=10; yy < img.rows-10; yy+=10)
        {
            if(img.at<unsigned char>(yy,xx)==255)
            {
                over_exp_block_cnt++;
                cv::circle(mask, cv::Point2f(xx,yy), 15, 0, -1);
            }
        }
    }

    vector<cv::Point2f>  features;
    cv::goodFeaturesToTrack(img, features, gftt_num, gftt_ql, gftt_dis, mask);
    vector<pair<cv::Point2f,float>> regionKeyPts_prepare[16];
    for(int i=0; i<16; i++)
    {
        regionKeyPts_prepare[i].clear();
    }
    fillIntoRegion(img,features,regionKeyPts_prepare,false);

    //pith up new features
    for(size_t i=0; i<16; i++)
    {
        sort(regionKeyPts_prepare[i].begin(), regionKeyPts_prepare[i].end(), sortbysecdesc);
        for(size_t j=0; j<regionKeyPts_prepare[i].size(); j++)
        {
            int noFeatureNearby = 1;
            cv::Point pt=regionKeyPts_prepare[i].at(j).first;
            for(size_t k=0; k<regionKeyPts[i].size(); k++)
            {
                float dis_x = fabs(pt.x-regionKeyPts[i].at(k).first.x);
                float dis_y = fabs(pt.y-regionKeyPts[i].at(k).first.y);
                if(dis_x <= boundary_dis || dis_y <= boundary_dis)
                {
                    noFeatureNearby=0;
                }
            }
            if(noFeatureNearby)
            {
                regionKeyPts[i].push_back(make_pair(pt,999999.0));
                new_features.push_back(pt);
                if(regionKeyPts[i].size() >= max_region_feature_num) break;
            }
        }
        //        if(regionKeyPts[i].size()<min_region_feature_num)
        //        {
        //            int cnt = min_region_feature_num-regionKeyPts[i].size();
        //            int x_begin = regionWidth*(i%4);
        //            int y_begin = regionHeight*(i/4);
        //            for(int i=0; i<cnt; i++)
        //            {
        //                int x=rand() % (regionWidth-10) + 5;
        //                int y=rand() % (regionHeight-10) + 5;
        //                cv::Point pt(x+x_begin,y+y_begin);

        //                new_features.push_back(pt);
        //            }
        //        }
    }

    //output
    if(new_features.size()>0)
    {
        //    trackedLMDescriptors.push_back(zeroDescriptor);
        for(size_t i=0; i<new_features.size(); i++)
        {
            newPts.push_back(Vec2(new_features.at(i).x,new_features.at(i).y));
        }
    }
}

void FeatureDEM::detect(const cv::Mat& img, vector<Vec2>& newPts)
{
    //Clear
    newPts.clear();

    vector<cv::Point2f>  features;
    cv::goodFeaturesToTrack(img,features, gftt_num*2, gftt_ql, gftt_dis);
    for(int i=0; i<16; i++)
    {
        regionKeyPts[i].clear();
    }
    fillIntoRegion(img,features,regionKeyPts,false);
    //For every region, select features by Harris index and boundary size
    for(int i=0; i<16; i++)
    {
        sort(regionKeyPts[i].begin(), regionKeyPts[i].end(), sortbysecdesc);
        vector<pair<cv::Point2f,float>> tmp = regionKeyPts[i];
        regionKeyPts[i].clear();
        int count = 0;
        for(size_t j=0; j<tmp.size(); j++)
        {
            int outSideConflictBoundary = 1;
            for(size_t k=0; k<regionKeyPts[i].size(); k++)
            {
                float dis_x = fabs(tmp.at(j).first.x-regionKeyPts[i].at(k).first.x);
                float dis_y = fabs(tmp.at(j).first.y-regionKeyPts[i].at(k).first.y);
                if(dis_x<=boundary_dis || dis_y<=boundary_dis)
                {
                    outSideConflictBoundary=0;
                }
            }
            if(outSideConflictBoundary)
            {
                regionKeyPts[i].push_back(tmp.at(j));
                count++;
                if(count>=max_region_feature_num) break;
            }
        }
    }
    //output
    features.clear();
    for(int i=0; i<16; i++)
    {
        //cout << regionKeyPts[i].size() << "in Region " << i << endl;
        for(size_t j=0; j<regionKeyPts[i].size(); j++)
        {
            features.push_back(regionKeyPts[i].at(j).first);
        }
    }
    //    trackedLMDescriptors.push_back(zeroDescriptor);
    for(size_t i=0; i<features.size(); i++)
    {
        newPts.push_back(Vec2(features.at(i).x,features.at(i).y));
    }
}



//void FeatureDEM::detect_conventional(const cv::Mat& img, vector<Vec2>& pts, vector<cv::Mat>& descriptors)
//{
//    //Clear
//    pts.clear();
//    descriptors.clear();
//    for(int i=0; i<16; i++)
//    {
//        regionKeyPts[i].clear();
//    }
//    //Detect FAST
//    cv::Ptr<cv::FastFeatureDetector> detector= cv::FastFeatureDetector::create();
//    //cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create(4000);
//    vector<cv::KeyPoint> tmpKPs;
//    detector->detect(img, tmpKPs);
//    //Fill into region
//    vector<cv::Point2f>  tmpPts;
//    cv::KeyPoint::convert(tmpKPs,tmpPts);
//    vector<cv::Point2f>  output;
//    output.clear();
//    int range=tmpPts.size();
//    for(int i=0; i<900; i++)
//    {
//        int idx = rand() % range;
//        output.push_back(tmpPts.at(idx));
//    }
//    cv::Mat tmpDescriptors;
//    cv::KeyPoint::convert(output,tmpKPs);
//    cv::Ptr<cv::DescriptorExtractor> extractor = cv::ORB::create();
//    extractor->compute(img, tmpKPs, tmpDescriptors);
//    for(size_t i=0; i<tmpKPs.size(); i++)
//    {
//        pts.push_back(Vec2(tmpKPs.at(i).pt.x,tmpKPs.at(i).pt.y));
//    }
//    descriptors_to_vMat(tmpDescriptors,descriptors);
//}
