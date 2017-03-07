#ifndef _VIRTUAL_CAMERA_H
#define _VIRTUAL_CAMERA_H


#include <cv.h>



namespace cvlab {

    class VirtualCameraImpl;
    class VirtualCamera 
    {

    public:
        explicit VirtualCamera( const cv::Mat _img, float fx=1000.0f, float fy=1000.0f );
        explicit VirtualCamera( IplImage* _img, float fx=1000.0f, float fy=1000.0f );
        ~VirtualCamera();

        void rotateOnXAxis( float angle );
        void rotateOnZAxis( float angle );
        void translate( float x, float y, float z );

        bool snap();
        const cv::Mat& getSnapshot() const { return snapshot; }
        cv::Mat getIntrinsics() const;
        cv::Mat getR() const;
        cv::Mat getT() const;
        bool saveToFiles( std::string dir_name, std::string prefix ) const;

    private:
        cv::Mat img;
        cv::Mat snapshot;
        VirtualCameraImpl* pImpl;
        
    };

}


#endif