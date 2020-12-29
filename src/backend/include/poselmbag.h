#ifndef POSELMBAG_H
#define POSELMBAG_H

#include <include/common.h>
#include <stdio.h>
#include <cstdint>

using namespace std;


struct LM_ITEM {
  int64_t id;
  int     count;
  Vec3    p3d_w;
};

struct POSE_ITEM {
  int64_t relevent_frame_id;
  int64_t pose_id;
  SE3     pose;
};


class PoseLMBag
{
public:
    vector<LM_ITEM>    lm_sub_bag;
    vector<POSE_ITEM>  pose_sub_bag;

    int             pose_buffer_size;
    int             newest;//idx of newest pose in pose_sub_bag
    int             oldest;//idx of oldest pose in pose_sub_bag
    int             wp_init;//write pointer for initialization process
    int             pose_cnt_init;//pose count for initialization process
    bool            pose_sub_bag_initialized;

    PoseLMBag(int pose_buffer_size_in);
    void reset(void);

    bool hasTheLM(int64_t id_in, int &idx);
    bool addLMObservation(int64_t id_in, Vec3 p3d_w_in);
    bool addLMObservationSlidingWindow(int64_t id_in, Vec3 p3d_w_in);
    bool removeLMObservation(int64_t id_in);

    void addPose(int64_t id_in, SE3 pose_in);//This will cover the oldest pose


    void getAllLMs(vector<LM_ITEM> &lms_out);
    void getMultiViewLMs(vector<LM_ITEM> &lms_out, int view_cnt=3);
    void getAllPoses(vector<POSE_ITEM> &poses_out);
    int getNewestPoseInOptimizerIdx(void);
    int getOldestPoseInOptimizerIdx(void);
    int64_t getPoseIdByReleventFrameId(int64_t frame_id);




    void debug_output(void);


private:

};

#endif // POSELMBAG_H
