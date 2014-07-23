/*
Copyright (c) 2012, Daniel Moreno and Gabriel Taubin
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Brown University nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL DANIEL MORENO AND GABRIEL TAUBIN BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "CalibrationData.hpp"

#include <QFileInfo>

#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>

CalibrationData::CalibrationData() :
    cam_K(), cam_kc(),
    proj_K(), proj_kc(),
    R(), T(),
    cam_error(0.0), proj_error(0.0), stereo_error(0.0),
    filename()
{
}

CalibrationData::~CalibrationData()
{
}

void CalibrationData::clear(void)
{
    cam_K = cv::Mat();
    cam_kc = cv::Mat();
    proj_K = cv::Mat();
    proj_kc = cv::Mat();
    R = cv::Mat();
    T = cv::Mat();
    filename = QString();
}

bool CalibrationData::is_valid(void) const
{
    return (cam_K.data && cam_kc.data && proj_K.data && proj_kc.data && R.data && T.data);
}

bool CalibrationData::load_calibration(QString const& filename)
{
    QFileInfo info(filename);
    QString type = info.suffix();

    if (type=="yml") {return load_calibration_yml(filename);}

    return false;
}

bool CalibrationData::save_calibration(QString const& filename)
{
    QFileInfo info(filename);
    QString type = info.suffix();

    if (type=="yml") {return save_calibration_yml(filename);}
    if (type=="m"  ) {return save_calibration_matlab(filename);}

    return false;
}

bool CalibrationData::load_calibration_yml(QString const& filename)
{
    cv::FileStorage fs(filename.toStdString(), cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        return false;
    }

    fs["cam_K"] >> cam_K;
    fs["cam_kc"] >> cam_kc;
    fs["proj_K"] >> proj_K;
    fs["proj_kc"] >> proj_kc;
    fs["R"] >> R;
    fs["T"] >> T;

    fs["cam_error"] >> cam_error;
    fs["proj_error"] >> proj_error;
    fs["stereo_error"] >> stereo_error;

    fs.release();

    this->filename = filename;

    return true;
}

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/core/utility.hpp"
#include "opencv2/core/core.hpp"

using namespace cv;

// NAC
// mostly from: https://github.com/Itseez/opencv/blob/master/samples/cpp/stereo_match.cpp
// FIXME: most of these only have to be computed once, due to rigidly mounted cam/projector
// FIXME: output parameters
void CalibrationData::init_rectification_maps(Size img_size) {

  //Size img_size = img.size();

  stereoRectify( cam_K, cam_kc, proj_K, proj_kc, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2 );

  initUndistortRectifyMap(cam_K, cam_kc, R1, P1, img_size, CV_16SC2, map11, map12);
  initUndistortRectifyMap(proj_K, proj_kc, R2, P2, img_size, CV_16SC2, map21, map22);
}

// NAC
void CalibrationData::rectify_pair(Mat& img1, Mat& img2) {
  
        Mat img1r, img2r;
        remap(img1, img1r, map11, map12, INTER_LINEAR);
        remap(img2, img2r, map21, map22, INTER_LINEAR);

        img1 = img1r;
        img2 = img2r;
}

// NAC
void CalibrationData::stereo_block_matching(Mat img1, Mat img2) {

    enum { STEREO_BM=0, STEREO_SGBM=1, STEREO_HH=2, STEREO_VAR=3 };
    int alg = STEREO_SGBM;
    int SADWindowSize = 0, numberOfDisparities = 0;
    //float scale = 1.f;

/*
    Ptr<StereoBM> bm = createStereoBM(16,9);
    Ptr<StereoSGBM> sgbm = createStereoSGBM(0,16,3);

    Size img_size = img1.size();

    numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_size.width/8) + 15) & -16;

    bm->setROI1(roi1);
    bm->setROI2(roi2);
    bm->setPreFilterCap(31);
    bm->setBlockSize(SADWindowSize > 0 ? SADWindowSize : 9);
    bm->setMinDisparity(0);
    bm->setNumDisparities(numberOfDisparities);
    bm->setTextureThreshold(10);
    bm->setUniquenessRatio(15);
    bm->setSpeckleWindowSize(100);
    bm->setSpeckleRange(32);
    bm->setDisp12MaxDiff(1);

    sgbm->setPreFilterCap(63);
    int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
    sgbm->setBlockSize(sgbmWinSize);

    int cn = img1.channels();

    sgbm->setP1(8*cn*sgbmWinSize*sgbmWinSize);
    sgbm->setP2(32*cn*sgbmWinSize*sgbmWinSize);
    sgbm->setMinDisparity(0);
    sgbm->setNumDisparities(numberOfDisparities);
    sgbm->setUniquenessRatio(10);
    sgbm->setSpeckleWindowSize(100);
    sgbm->setSpeckleRange(32);
    sgbm->setDisp12MaxDiff(1);
    sgbm->setMode(alg == STEREO_HH ? StereoSGBM::MODE_HH : StereoSGBM::MODE_SGBM);

*/

/*

    int64 t = getTickCount();
    if( alg == STEREO_BM )
        bm->compute(img1, img2, disp);
    else if( alg == STEREO_SGBM || alg == STEREO_HH )
        sgbm->compute(img1, img2, disp);
    t = getTickCount() - t;
    printf("Time elapsed: %fms\n", t*1000/getTickFrequency());

    //disp = dispp.colRange(numberOfDisparities, img1p.cols);
    if( alg != STEREO_VAR )
        disp.convertTo(disp8, CV_8U, 255/(numberOfDisparities*16.));
    else
        disp.convertTo(disp8, CV_8U);

    if(disparity_filename)
        imwrite(disparity_filename, disp8);

*/

}

// NAC
void CalibrationData::disparityImg2Cloud(Mat disp, const char* point_cloud_filename) {
    if(point_cloud_filename)
    {
        printf("storing the point cloud...");
        fflush(stdout);
        Mat xyz;
        reprojectImageTo3D(disp, xyz, Q, true);
        saveXYZ(point_cloud_filename, xyz);
        printf("\n");
    }
}

// NAC
// helper
void CalibrationData::saveXYZ(const char* filename, const Mat& mat)
{
    const double max_z = 1.0e4;
    FILE* fp = fopen(filename, "wt");
    for(int y = 0; y < mat.rows; y++)
    {
        for(int x = 0; x < mat.cols; x++)
        {
            Vec3f point = mat.at<Vec3f>(y, x);
            if(fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;
            fprintf(fp, "%f %f %f\n", point[0], point[1], point[2]);
        }
    }
    fclose(fp);
}


bool CalibrationData::save_calibration_yml(QString const& filename)
{
    cv::FileStorage fs(filename.toStdString(), cv::FileStorage::WRITE);
    if (!fs.isOpened())
    {
        return false;
    }

    fs << "cam_K" << cam_K << "cam_kc" << cam_kc
       << "proj_K" << proj_K << "proj_kc" << proj_kc
       << "R" << R << "T" << T
       << "cam_error" << cam_error
       << "proj_error" << proj_error
       << "stereo_error" << stereo_error
       ;
    fs.release();

    this->filename = filename;

    return true;
}

bool CalibrationData::save_calibration_matlab(QString const& filename)
{
    FILE * fp = fopen(qPrintable(filename), "w");
    if (!fp)
    {
        return false;
    }

    cv::Mat rvec;
    cv::Rodrigues(R, rvec);
    fprintf(fp, 
        "%% Projector-Camera Stereo calibration parameters:\n"
        "\n"
        "%% Intrinsic parameters of camera:\n"
        "fc_left = [ %lf %lf ]; %% Focal Length\n"
        "cc_left = [ %lf %lf ]; %% Principal point\n"
        "alpha_c_left = [ %lf ]; %% Skew\n"
        "kc_left = [ %lf %lf %lf %lf %lf ]; %% Distortion\n"
        "\n"
        "%% Intrinsic parameters of projector:\n"
        "fc_right = [ %lf %lf ]; %% Focal Length\n"
        "cc_right = [ %lf %lf ]; %% Principal point\n"
        "alpha_c_right = [ %lf ]; %% Skew\n"
        "kc_right = [ %lf %lf %lf %lf %lf ]; %% Distortion\n"
        "\n"
        "%% Extrinsic parameters (position of projector wrt camera):\n"
        "om = [ %lf %lf %lf ]; %% Rotation vector\n"
        "T = [ %lf %lf %lf ]; %% Translation vector\n",
        cam_K.at<double>(0,0), cam_K.at<double>(1,1), cam_K.at<double>(0,2), cam_K.at<double>(1,2), cam_K.at<double>(0,1),
        cam_kc.at<double>(0,0), cam_kc.at<double>(0,1), cam_kc.at<double>(0,2), cam_kc.at<double>(0,3), cam_kc.at<double>(0,4), 
        proj_K.at<double>(0,0), proj_K.at<double>(1,1), proj_K.at<double>(0,2), proj_K.at<double>(1,2), proj_K.at<double>(0,1),
        proj_kc.at<double>(0,0), proj_kc.at<double>(0,1), proj_kc.at<double>(0,2), proj_kc.at<double>(0,3), proj_kc.at<double>(0,4),
        rvec.at<double>(0,0), rvec.at<double>(1,0), rvec.at<double>(2,0), 
        T.at<double>(0,0), T.at<double>(1,0), T.at<double>(2,0)
        );
    fclose(fp);

    return true;
}

void CalibrationData::display(std::ostream & stream) const
{
    stream << "Camera Calib: " << std::endl
        << " - reprojection error: " << cam_error << std::endl
        << " - K:\n" << cam_K << std::endl
        << " - kc: " << cam_kc << std::endl
        ;
    stream << std::endl;
    stream << "Projector Calib: " << std::endl
        << " - reprojection error: " << proj_error << std::endl
        << " - K:\n" << proj_K << std::endl
        << " - kc: " << proj_kc << std::endl
        ;
    stream << std::endl;
    stream << "Stereo Calib: " << std::endl
        << " - reprojection error: " << stereo_error << std::endl
        << " - R:\n" << R << std::endl
        << " - T:\n" << T << std::endl
        ;
}
