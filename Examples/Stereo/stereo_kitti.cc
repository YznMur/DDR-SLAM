/**
* This file is a modified version of ORB-SLAM2.<https://github.com/raulmur/ORB_SLAM2>
*
* This file is part of DDRSLAM.
** For more information see <https://github.com/YznMur/ddr_orb2.git>.
*
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>
#include <unistd.h>
#include<opencv2/core/core.hpp>

#include "Geometry.h"
#include<System.h>

using namespace std;

extern cv::Mat ddrnetPredict( cv::Mat &img, char * ptr, size_t size );
extern void cudaRun();

char * readFile(std::string name, size_t &size){
    char * trtModelStream;
    // loading input model ---------------------------
    std::ifstream file(name, std::ios::binary);
    if (file.good()) {
        file.seekg(0, file.end);
        size = file.tellg();
        file.seekg(0, file.beg);
        trtModelStream = new char[size];
        assert(trtModelStream);
        file.read(trtModelStream, size);
        file.close();
        return trtModelStream;
    }
    
    return nullptr;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    if(argc != 4  && argc != 5)
    {
        cerr << endl << "Usage: ./stereo_kitti path_to_vocabulary path_to_settings path_to_sequence (path_to_masks)" << endl;
        return 1;
    }
    size_t size_engine;
    char *data_engine = readFile("/root/ddr_orb2/ddrnet_trt/buildExec/DDRNet.engine", size_engine);

    std::cout << "size engine:" << size_engine << std::endl;
    std::cout << "data engine:" << data_engine << std::endl;

    // Retrieve paths to images
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimestamps;
    LoadImages(string(argv[3]), vstrImageLeft, vstrImageRight, vTimestamps);

    const int nImages = vstrImageLeft.size();

    cout << "Loading DDRNet weights. This could take a while..." << endl;


    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;   

    // Main loop
    cv::Mat imLeft, imRight;

    // Dilation settings
    int dilation_size = 5;
    cv::Mat kernel = getStructuringElement(cv::MORPH_ELLIPSE,
                                        cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                        cv::Point( dilation_size, dilation_size ) );

    cudaRun();

    for(int ni=0; ni<nImages; ni++)
    {
        // Read left and right images from file
        imLeft = cv::imread(vstrImageLeft[ni],CV_LOAD_IMAGE_UNCHANGED);
        imRight = cv::imread(vstrImageRight[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(imLeft.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(vstrImageLeft[ni]) << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        int h = imLeft.rows;
        int w = imLeft.cols;
        cv::Mat maskRight = cv::Mat::ones(h,w,CV_8U);
        cv::Mat maskLeft = cv::Mat::ones(h,w,CV_8U);

        // Segment out the images
        if (argc == 5){

            // std::cout << "first"<< std::endl;

            cv::Mat maskLeftRCNNRgb, maskRightRCNNRgb;
            maskLeftRCNNRgb = ddrnetPredict( imLeft, data_engine, size_engine );
            maskRightRCNNRgb = ddrnetPredict( imRight, data_engine, size_engine );

            cv::Mat maskLeftRCNN, maskRightRCNN;

            cv::Mat bgr[3];
            cv::split(maskLeftRCNNRgb,bgr);
            maskLeftRCNN = bgr[0];

            cv::split(maskRightRCNNRgb,bgr);
            maskRightRCNN = bgr[0];
            cv::Mat maskLeftRCNNdil = maskLeftRCNN.clone();
            cv::dilate(maskLeftRCNN, maskLeftRCNNdil, kernel);
            maskLeft = maskLeft - maskLeftRCNNdil;
            cv::Mat maskRightRCNNdil = maskRightRCNN.clone();
            cv::dilate(maskRightRCNN, maskRightRCNNdil, kernel);
            maskRight = maskRight - maskRightRCNNdil;
        }

        // Pass the images to the SLAM system
        SLAM.TrackStereo(imLeft, imRight, maskLeft, maskRight, tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");

    return 0;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/image_2/";
    string strPrefixRight = strPathToSequence + "/image_3/";

    const int nTimes = vTimestamps.size();
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
        vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
    }
}
