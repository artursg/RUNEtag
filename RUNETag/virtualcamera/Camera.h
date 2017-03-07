#ifndef _CAMERA_H
#define _CAMERA_H

#include <cv.h>

class Camera {

public:
    Camera( );
    Camera( const cv::Mat& _R, const cv::Mat& _T, const cv::Mat& _intrinsics );
    Camera( const cv::Mat& _R, const cv::Mat& _T, float fx, float fy, float cx, float cy );

    inline const double& fx() const { return intrinsics.at<double>(0,0); }
    inline const double& fy() const { return intrinsics.at<double>(1,1); }
    inline const double& cx() const { return intrinsics.at<double>(0,2); }
    inline const double& cy() const { return intrinsics.at<double>(1,2); }

    inline const cv::Mat& getR()  const { return R; }
    inline const cv::Mat& getT()  const { return T; }
    inline const cv::Mat& getIntrinsics() const { return intrinsics; }

    cv::Point2d world2Image( const cv::Point3d& p ) const;
    cv::Point3d image2xy( const cv::Point2i& pi ) const;
    void applyRT( cv::Mat& p ) const;
    void applyInverseRT( cv::Mat& p ) const;

private:
    cv::Mat R;
    cv::Mat T;
    cv::Mat intrinsics;
};



#endif