#include "Camera.h"


Camera::Camera( ) : R( cv::Mat::eye(3,3,CV_64F) ), T( cv::Mat(3,1,CV_64F) ) {
}

Camera::Camera( const cv::Mat& _R, const cv::Mat& _T, const cv::Mat& _intrinsics ) : R(_R.clone()), T(_T.clone()), intrinsics(_intrinsics.clone() ) {

}


Camera::Camera( const cv::Mat& _R, const cv::Mat& _T, float fx, float fy, float cx, float cy ) : R(_R.clone()), T(_T.clone()), intrinsics( cv::Mat(3,3,CV_64F) ) {
    intrinsics.at<double>(0,0) = fx;
    intrinsics.at<double>(1,1) = fy;

    intrinsics.at<double>(0,2) = cx;
    intrinsics.at<double>(1,2) = cy;

    intrinsics.at<double>(2,2) = 1.0;
}



cv::Point2d Camera::world2Image( const cv::Point3d& p ) const {

    cv::Mat pv(3,1,CV_64F);
    pv.at<double>(0,0) = p.x;
    pv.at<double>(1,0) = p.y;
    pv.at<double>(2,0) = p.z;

    pv = R*pv;
    pv.at<double>(0,0) += T.at<double>(0,0);
    pv.at<double>(1,0) += T.at<double>(1,0);
    pv.at<double>(2,0) += T.at<double>(2,0);

    pv.at<double>(0,0) /= pv.at<double>(2,0);
    pv.at<double>(1,0) /= pv.at<double>(2,0);
    pv.at<double>(2,0) = 1.0;


    return cv::Point2d( pv.at<double>(0,0) * fx() + cx(), pv.at<double>(1,0) * fy() + cy() );
}

void Camera::applyRT( cv::Mat& p ) const {
    p = R*p;
    p = p + T;
}

void Camera::applyInverseRT( cv::Mat& p ) const {
    cv::Mat Rinv = R.clone();
    cv::transpose( R, Rinv );
    cv::Mat Tinv = Rinv*T;
    Tinv.at<double>(0,0) *= -1.0;
    Tinv.at<double>(1,0) *= -1.0;
    Tinv.at<double>(2,0) *= -1.0;

    p = Rinv*p;
    p = p + Tinv;
}

cv::Point3d Camera::image2xy( const cv::Point2i& pi ) const {
    return cv::Point3d( (pi.x - cx()) / fx(), (pi.y - cy()) / fy(), 1.0 );
}