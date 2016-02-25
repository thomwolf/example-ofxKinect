/*
 * KinectGrabber.cpp
 *
 *  Created on: Oct 8, 2014
 *      Author: arturo
 */

#include "KinectGrabber.h"
#include "ofConstants.h"

KinectGrabber::KinectGrabber()
:newFrame(true){
	// start the thread as soon as the
	// class is created, it won't use any CPU
	// until we send a new frame to be analyzed
    //	startThread();
}

KinectGrabber::~KinectGrabber(){
	// when the class is destroyed
	// close both channels and wait for
	// the thread to finish
    //	toAnalyze.close();
    //	analyzed.close();
	waitForThread(true);
}

void KinectGrabber::setup(){
    //	// send the frame to the thread for analyzing
    //	// this makes a copy but we can't avoid it anyway if
    //	// we want to update the grabber while analyzing
    //    // previous frames
    
	// settings and defaults
	enableCalibration = false;
	enableTestmode	  = true;
	storedframes = 0;
    //    storedcoloredframes = 0;
    
    kinect.init();
    kinect.setRegistration(true);
    kinect.open();
    kinect.setUseTexture(false);
    kinectWidth = kinect.getWidth();
    kinectHeight = kinect.getHeight();
    kinectDepthImage.allocate(kinectWidth, kinectHeight);
    kinectDepthImage.setUseTexture(false);
    kinectColorImage.allocate(kinectWidth, kinectHeight);
    kinectColorImage.setUseTexture(false);
}

void KinectGrabber::setupFramefilter(int sNumAveragingSlots, unsigned int newMinNumSamples, unsigned int newMaxVariance, float newHysteresis, bool newSpatialFilter, int gradFieldresolution, float snearclip, float sfarclip) {
    framefilter.setup(kinectWidth, kinectHeight, sNumAveragingSlots, newMinNumSamples, newMaxVariance, newHysteresis, newSpatialFilter, gradFieldresolution, snearclip, sfarclip);
    // framefilter.startThread();
}

void KinectGrabber::setupCalibration(int projectorWidth, int projectorHeight, float schessboardSize, float schessboardColor, float sStabilityTimeInMs, float smaxReprojError){
    
    // make the wrapper (to make calibration independant of the drivers...)
    kinectWrapper = new RGBDCamCalibWrapperOfxKinect();
    kinectWrapper->setup(&kinect);
    kinectProjectorCalibration.setup(kinectWrapper, projectorWidth, projectorHeight);
    
    // some default config
    kinectProjectorCalibration.chessboardSize = schessboardSize;
    kinectProjectorCalibration.chessboardColor = schessboardColor;
    kinectProjectorCalibration.setStabilityTimeInMs(sStabilityTimeInMs);
    maxReprojError = smaxReprojError;
    
    // sets the output
    kinectProjectorOutput.setup(kinectWrapper, projectorWidth, projectorHeight);
    kinectProjectorOutput.load("kinectProjector.yml");
}

void KinectGrabber::setupClip(float snearclip, float sfarclip){
    //	// send the frame to the thread for analyzing
    //	// this makes a copy but we can't avoid it anyway if
    //	// we want to update the grabber while analyzing
    //    // previous frames
    
    //    if (framefilter.isThreadRunning()){
    //    }
}

void KinectGrabber::setTestmode(){
    enableTestmode = true;
    enableCalibration = false;
    kinectProjectorOutput.load("kinectProjector.yml");
}

void KinectGrabber::setCalibrationmode(){
    enableCalibration = true;
    enableTestmode = false;
    //    kinectProjectorOutput.load("kinectProjector.yml");
}

bool KinectGrabber::isFrameNew(){
	return newFrame;
}

void KinectGrabber::threadedFunction(){
    // wait until there's a new frame
    // this blocks the thread, so it doesn't use
    // the CPU at all, until a frame arrives.
    // also receive doesn't allocate or make any copies
	while(isThreadRunning()) {
        newFrame = false;
        if (storedframes == 0)
        {
            // If new image in kinect => send to filter thread
            kinect.update();
            
            if(kinect.isFrameNew()){
                newFrame = true;
                //		kinectColorImage.setFromPixels(kinect.getPixels());
                kinectDepthImage.setFromPixels(kinect.getDepthPixels());
                //		kinectColoredDepth.setFromPixels(kinectDepthImage.getPixels());
                
                if (enableCalibration) {
                    kinectColorImage.setFromPixels(kinect.getPixels());
                    // If new filtered image => send back to main thread
#if __cplusplus>=201103
                    colored.send(std::move(kinectColorImage.getPixels()));
                    filtered.send(std::move(kinectDepthImage.getPixels()));
#else
                    colored.send(kinectColorImage.getPixels());
                    filtered.send(kinectDepthImage.getPixels());
#endif
                    lock();
                    storedframes += 1;
                    unlock();
                    
                }
                // if the test mode is activated, the settings are loaded automatically (see gui function)
                if (enableTestmode) {
                    ofPixels filteredframe;
                    filteredframe = framefilter.filter(kinectDepthImage.getPixels());
                    filteredframe.setImageType(OF_IMAGE_GRAYSCALE);
                    
                    // If new filtered image => send back to main thread
#if __cplusplus>=201103
                    filtered.send(std::move(filteredframe));
                    gradient.send(std::move(framefilter.getGradField()));
#else
                    filtered.send(filteredframe);
                    gradient.send(framefilter.getGradField());
#endif
                    lock();
                    storedframes += 1;
                    unlock();
                }
            }
        }
        
        //Update clipping planes of kinect if needed
        float snearclip = nearclip;
        float sfarclip = farclip;
        if(nearclipchannel.tryReceive(snearclip) || farclipchannel.tryReceive(sfarclip)) {
            while(nearclipchannel.tryReceive(snearclip) || farclipchannel.tryReceive(sfarclip)) {
            } // clear queue
            kinect.setDepthClipping(snearclip, sfarclip);
            framefilter.setDepthRange(snearclip, sfarclip);
            framefilter.resetBuffers();
        
        }
    }
    kinect.close();
}

