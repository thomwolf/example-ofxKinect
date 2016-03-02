#pragma once
#include "ofMain.h"

#include "ofxOpenCv.h"
#include "ofxCv.h"
#include "ofxKinect.h"

#include "FrameFilter.h"

class KinectGrabber: public ofThread {
public:
	KinectGrabber();
	~KinectGrabber();
    void setup();
    void setupClip(float nearclip, float farclip);
    void setupFramefilter(int sNumAveragingSlots, unsigned int newMinNumSamples, unsigned int newMaxVariance, float newHysteresis, bool newSpatialFilter, int gradFieldresolution,float snearclip, float sfarclip);
    void setupCalibration(int projectorWidth, int projectorHeight, float schessboardSize, float schessboardColor, float sStabilityTimeInMs, float smaxReprojError);
    void setCalibrationmode();
    void setTestmode();
    //void update();
//    ofPixels convertProjSpace(ofPixels sinputframe);
	bool isFrameNew();
    int storedframes;//, storedcoloredframes;
	ofPixels & getPixels();
	ofTexture & getTexture();
    
	ofThreadChannel<ofPixels> filtered;
	ofThreadChannel<ofPixels> colored;
	ofThreadChannel<ofVec2f*> gradient;
	ofThreadChannel<float> nearclipchannel;
	ofThreadChannel<float> farclipchannel;

    ofxKinect               kinect;
//    float                       lowThresh;
//    float                       highThresh;
    float                       chessboardThreshold;
    // Framefilter
    FrameFilter                 framefilter;

private:
	void threadedFunction();
//	ofThreadChannel<ofPixels> toAnalyze;
	ofPixels pixels;
	ofTexture texture;
	bool newFrame;
	bool enableCalibration, enableTestmode;
    
    // kinect & the wrapper
    float                   nearclip, farclip;
    int kinectWidth, kinectHeight;//, projWidth, projHeight;
    ofxCvColorImage         kinectColorImage;
//    ofxCvGrayscaleImage		kinectGreyscaledImage;
    ofxCvGrayscaleImage     kinectDepthImage;
    //   ofImage                 kinectColoredDepth;
    float maxReprojError;
    // calibration
    // output
};
