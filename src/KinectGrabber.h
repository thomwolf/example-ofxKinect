#pragma once
#include "ofMain.h"

#include "ofxOpenCv.h"
#include "ofxCv.h"
#include "ofxKinect.h"

#include "RGBDCamCalibWrapperOfxKinect.h"
#include "ofxKinectProjectorCalibration.h"
#include "ofxXmlSettings.h"

#include "FrameFilter.h"

class KinectGrabber: public ofThread {
public:
	KinectGrabber();
	~KinectGrabber();
    void setup();
    void setupClip(float nearclip, float farclip);
    void setupFramefilter(int sNumAveragingSlots, unsigned int newMinNumSamples, unsigned int newMaxVariance, float newHysteresis, bool newSpatialFilter, int gradFieldresolution);
    void setupCalibration(int projectorWidth, int projectorHeight, float schessboardSize, float schessboardColor, float sStabilityTimeInMs, float smaxReprojError);
    void setCalibrationmode();
    void setTestmode();
    //void update();
	bool isFrameNew();
    int storedframes;//, storedcoloredframes;
	ofPixels & getPixels();
	ofTexture & getTexture();
    
	ofThreadChannel<ofPixels> filtered;
	ofThreadChannel<ofPixels> colored;
	ofThreadChannel<float> nearclipchannel;
	ofThreadChannel<float> farclipchannel;

    KinectProjectorCalibration	kinectProjectorCalibration;
//    float                       lowThresh;
//    float                       highThresh;
    float                       chessboardThreshold;

private:
	void threadedFunction();
//	ofThreadChannel<ofPixels> toAnalyze;
	ofPixels pixels;
	ofTexture texture;
	bool newFrame;
	bool enableCalibration, enableTestmode;
    
    // kinect & the wrapper
    ofxKinect               kinect;
    float                   nearclip, farclip;
    int kinectWidth, kinectHeight;
    ofxCvColorImage         kinectColorImage;
//    ofxCvGrayscaleImage		kinectGreyscaledImage;
    ofxCvGrayscaleImage     kinectDepthImage;
    //   ofImage                 kinectColoredDepth;
    float maxReprojError;
    // calibration
    RGBDCamCalibWrapper*	kinectWrapper;
    // output
    KinectProjectorOutput	kinectProjectorOutput;
    
    // Framefilter thread
    FrameFilter                 framefilter;
};
