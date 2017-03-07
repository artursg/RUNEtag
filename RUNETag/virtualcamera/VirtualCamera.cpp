#include "VirtualCamera.h"
#include "Camera.h"

using namespace cvlab;

class cvlab::VirtualCameraImpl {
public:
    VirtualCameraImpl( cv::Mat src_img, float _fx, float _fy ) : R( cv::Mat::eye(3,3,CV_64F) ), T(cv::Mat(3,1,CV_64F)), fx(_fx), fy(_fy), cx( src_img.cols/2.0f ), cy( src_img.rows/2.0f ), xangle(0.0f), zangle(0.0f) {
        T.at<double>(0,0) = 0.0;
        T.at<double>(1,0) = 0.0;
        T.at<double>(2,0) = fx;
    }
    float fx;
    float fy;
    float cx;
    float cy;
    float xangle;
    float zangle;
    cv::Mat R;
    cv::Mat T;
};



//
//  Aux Functions
//

void rotateX( cv::Mat m,  double angle ) {

    cv::Mat r(3,3,CV_64F);
    r.at<double>(0,0) = 1.0;
    r.at<double>(0,1) = 0.0;
    r.at<double>(0,2) = 0.0;

    r.at<double>(1,0) = 0.0;
    r.at<double>(1,1) = cos(angle);
    r.at<double>(1,2) = -sin(angle);

    r.at<double>(2,0) = 0.0;
    r.at<double>(2,1) = sin(angle);
    r.at<double>(2,2) = cos(angle);
    m = r*m;
}

void rotateY( cv::Mat m,  double angle ) {

    cv::Mat r(3,3,CV_64F);
    r.at<double>(0,0) = cos(angle);
    r.at<double>(0,1) = 0.0;
    r.at<double>(0,2) = sin(angle);

    r.at<double>(1,0) = 0.0;
    r.at<double>(1,1) = 1.0;
    r.at<double>(1,2) = 0.0;

    r.at<double>(2,0) = -sin(angle);
    r.at<double>(2,1) = 0.0;
    r.at<double>(2,2) = cos(angle);
    m = r*m;
}

void rotateZ( cv::Mat m,  double angle ) {

    cv::Mat r(3,3,CV_64F);
    r.at<double>(0,0) = cos(angle);
    r.at<double>(0,1) = -sin(angle);
    r.at<double>(0,2) = 0.0;

    r.at<double>(1,0) = sin(angle);
    r.at<double>(1,1) = cos(angle);
    r.at<double>(1,2) = 0.0;

    r.at<double>(2,0) = 0.0;
    r.at<double>(2,1) = 0.0;
    r.at<double>(2,2) = 1.0;
    m = r*m;
}

double dot( const cv::Mat& v1, const cv::Mat& v2 ) {
    double sum = 0.0;
    for( int i=0; i< v1.rows; i++ ) {
        sum += v1.at<double>(i,0)*v2.at<double>(i,0);
    }
    return sum; 
}



inline bool segmentPlaneIntersect( cv::Mat p0, cv::Mat p1, cv::Mat plane_origin, cv::Mat plane_normal, cv::Mat& intersection ) {

    cv::Mat u = p1 - p0;
    cv::Mat w = p0 - plane_origin;

    double  D = dot( plane_normal, u );
    double  N = -dot( plane_normal, w);

    if (fabs(D) < 0.0001) {             // segment is parallel to plane
        return false;
    }

    // they are not parallel
    // compute intersect param
    double sI = N / D;
   /* if (sI < 0 || sI > 1)
        return 0;                       // no intersection
    */

    intersection = p0 + sI * u;         // compute segment intersect point
    return true;
}



//
//  VirtualCamera implementation
//

cvlab::VirtualCamera::VirtualCamera( const cv::Mat _img, float fx, float fy ) : pImpl( new VirtualCameraImpl(_img,fx,fy) ), img( _img.clone() ) {}


cvlab::VirtualCamera::VirtualCamera( IplImage* _img, float fx, float fy ) : pImpl( new VirtualCameraImpl(_img,fx,fy) ), img( cv::Mat(_img).clone() ) {}

VirtualCamera::~VirtualCamera() {
    delete pImpl;
}

cv::Mat VirtualCamera::getIntrinsics() const {
    cv::Mat intrinsics = cv::Mat::eye(3,3,CV_64F);
    intrinsics.at<double>(0,0) = pImpl->fx;
    intrinsics.at<double>(1,1) = pImpl->fy;
    intrinsics.at<double>(0,2) = pImpl->cx;
    intrinsics.at<double>(1,2) = pImpl->cy;
    return intrinsics;
}


cv::Mat VirtualCamera::getR() const {
    cv::Mat rflip = pImpl->R.clone();
    /*
    rflip.at<double>(0,1) *= -1.0;
    rflip.at<double>(1,1) *= -1.0;
    rflip.at<double>(2,1) *= -1.0;
    rflip.at<double>(0,2) *= -1.0;
    rflip.at<double>(1,2) *= -1.0;
    rflip.at<double>(2,2) *= -1.0;
    */
    return rflip;
}


cv::Mat VirtualCamera::getT() const {
    return pImpl->T;
}


void VirtualCamera::rotateOnXAxis( float angle ) {
    pImpl->xangle += angle;
}


void VirtualCamera::rotateOnZAxis( float angle ) {
    pImpl->zangle += angle;
}

void VirtualCamera::translate( float x, float y, float z ) {
    pImpl->T.at<double>(0,0) += x;
    pImpl->T.at<double>(1,0) += y;
    pImpl->T.at<double>(2,0) += z;
}

bool VirtualCamera::snap() {

    cv::Mat mapx( img.rows, img.cols, CV_32FC1 );
    cv::Mat mapy( img.rows, img.cols, CV_32FC1 );

    snapshot = img.clone();

    pImpl->R = cv::Mat::eye(3,3,CV_64F);
    rotateZ( pImpl->R, pImpl->zangle );
    rotateX( pImpl->R, pImpl->xangle );
    Camera cam( pImpl->R, pImpl->T, pImpl->fx, pImpl->fy, pImpl->cx, pImpl->cy );

    cv::Mat cam_origin(3,1,CV_64F);
    cv::Mat cam_dir(3,1,CV_64F);
    cv::Mat world_plane_origin(3,1,CV_64F);
    cv::Mat world_plane_dir(3,1,CV_64F);

    
    for( int y=0; y<snapshot.rows; y++ ) {
        for( int x=0; x<snapshot.cols; x++ ) {

            cv::Point2i p( x , y  );
            cv::Point3d p_xy = cam.image2xy( p );
            
            cam_origin.at<double>(0,0) = 0.0;
            cam_origin.at<double>(1,0) = 0.0;
            cam_origin.at<double>(2,0) = 0.0;
            
            cam_dir.at<double>(0,0) = p_xy.x;
            cam_dir.at<double>(1,0) = p_xy.y;
            cam_dir.at<double>(2,0) = p_xy.z;
            
            world_plane_origin.at<double>(0,0) = 0.0;
            world_plane_origin.at<double>(1,0) = 0.0;
            world_plane_origin.at<double>(2,0) = 0.0;
            
            world_plane_dir.at<double>(0,0) = 0.0;
            world_plane_dir.at<double>(1,0) = 0.0;
            world_plane_dir.at<double>(2,0) = 1.0;

            cam.applyInverseRT( cam_origin );
            cam.applyInverseRT( cam_dir );

            cv::Mat intersection;
            if( segmentPlaneIntersect( cam_origin, cam_dir, world_plane_origin, world_plane_dir, intersection ) ){
                mapx.at<float>(y,x) = static_cast<float>( intersection.at<double>(0,0) + img.cols/2.0 );
                mapy.at<float>(y,x) = static_cast<float>( intersection.at<double>(1,0) + img.rows/2.0 );
            }
        }
    }

    cv::remap( img, snapshot, mapx, mapy, cv::INTER_LINEAR );
    return true;
}



bool VirtualCamera::saveToFiles( std::string dir_name, std::string prefix ) const {
    //Not implemented yet
    return false;
}
