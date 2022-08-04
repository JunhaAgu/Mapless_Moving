#ifndef _OBJECT_EXT_H_
#define _OBJECT_EXT_H_

#include "defines.h"
#include "user_param.h"
#include "cloud_frame.h"

class MaplessDynamic;
class UserParam;

class ObjectExt
{
private:
    int img_height_;
    int img_width_;

    int thr_object_;
    float alpha_;
    float beta_;
public:


public:
    ObjectExt(const std::unique_ptr<UserParam> &user_param);
    ~ObjectExt();

    void filterOutAccumdR(StrRhoPts *str_next, StrRhoPts *str_cur_warped, cv::Mat &accumulated_dRdt, cv::Mat &accumulated_dRdt_score, cv::Mat &residual);
    void extractObjectCandidate(cv::Mat &accumulated_dRdt, StrRhoPts *str_next);

    void checkSegment(cv::Mat &accumulated_dRdt, StrRhoPts *str_next, cv::Mat &groundPtsIdx_next);

    void updateScore(cv::Mat &accumulated_dRdt, cv::Mat &accumulated_dRdt_score);
};

#endif