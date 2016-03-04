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
    kinect.setRegistration(true); // So we have correspondance between RGB and depth images
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
    nearclip =snearclip;
    farclip =sfarclip;
    kinect.setDepthClipping(snearclip, sfarclip);
    framefilter.setup(kinectWidth, kinectHeight, sNumAveragingSlots, newMinNumSamples, newMaxVariance, newHysteresis, newSpatialFilter, gradFieldresolution, snearclip, sfarclip, &kinect);
    // framefilter.startThread();
}

void KinectGrabber::setupClip(float snearclip, float sfarclip){
    //	// send the frame to the thread for analyzing
    //	// this makes a copy but we can't avoid it anyway if
    //	// we want to update the grabber while analyzing
    //    // previous frames
    
    //    if (framefilter.isThreadRunning()){
    //    }
    nearclip =snearclip;
    farclip =sfarclip;
    kinect.setDepthClipping(snearclip, sfarclip);
}

void KinectGrabber::setTestmode(){
    enableTestmode = true;
    enableCalibration = false;
}

void KinectGrabber::setCalibrationmode(){
    enableCalibration = true;
    enableTestmode = false;
    //    kinectProjectorOutput.load("kinectProjector.yml");
}

bool KinectGrabber::isFrameNew(){
	return newFrame;
}

//ofPixels KinectGrabber::convertProjSpace(ofPixels inputframe){
//    // Create a new output frame: */
//    ofPixels newOutputFrame;
//    newOutputFrame.allocate(projWidth, projHeight, 1);
//    newOutputFrame.set(0);
//    
////    unsigned char* ifPtr=static_cast<unsigned char*>(inputframe.getData());
////    unsigned char* nofPtr=static_cast<unsigned char*>(newOutputFrame.getData());
////    
////    ofPoint v1, v2; // v1.x is 0, v1.y is 0, v1.z is 0
////    float z;
////    int ind, val;
////    
////    for(unsigned int y=0;y<kinectHeight;y = y + 1)
////    {
////        for(unsigned int x=0;x<kinectWidth;x = x + 1)
////        {
////            //float z  = farclip;//+nearclip)/2;
////            //cout << "iptr: " << (int)ifPtr[y*kinectWidth+x] << endl;
////            val = ifPtr[y*kinectWidth+x];
////            if (val != 0 && val != 255) {
////                z = (255.0-(float)val)/255.0*(farclip-nearclip)+nearclip;
////                v1.set(x, y, z);// = ofPoint(
////                v2 = kinectProjectorOutput.projectFromDepthXYZ(v1);
//////                cout << "v1: " << v1 << endl;
//////                cout << "v2: " << v2 << endl;
////                if (v2.y >= 0 && v2.y < 600 && v2.x >=0 && v2.x < 800) {
////                    ind = (int)floorf(v2.y)*800+(int)floorf(v2.x);
////                    nofPtr[ind]=val;
////                }
////            }
////        }
////    }
//        newOutputFrame = inputframe;
//    return newOutputFrame;
//}

//ofSetColor(255, 190, 70);
//ofPoint cent = ofPoint(projectorWidth/2, projectorHeight/2);
//for (int i = 0; i < contourFinder.size(); i++) {
//
//    ofPolyline blobContour = contourFinder.getPolyline(i);
//    if(!blobContour.isClosed()){
//        blobContour.close();
//    }
//
//    //if (!blobContour.inside(cent)) {
//    ofPolyline rect = blobContour.getResampledByCount(8);
//    ofBeginShape();
//    kinectgrabber.lock();
//    for (int j = 0; j < rect.size() - 1; j++) {
//        rect[j].z = (farclip-nearclip)*highThresh+nearclip;
//        ofPoint wrld = kinectgrabber.kinectWrapper->getWorldFromRgbCalibratedXYZ(rect[j], true,true);
//        ofPoint currVertex = kinectgrabber.kinectProjectorOutput.projectFromDepthXYZ(rect[j]);
//        ofVertex(currVertex.x, currVertex.y);
//        //					cout << "blob j: "<< j << " rect: "<< rect[j] << " wrld: " << wrld << " currVertex : " << currVertex << endl;
//    }
//    kinectgrabber.unlock();
//    ofEndShape();
//    //}

void KinectGrabber::threadedFunction(){
    // wait until there's a new frame
    // this blocks the thread, so it doesn't use
    // the CPU at all, until a frame arrives.
    // also receive doesn't allocate or make any copies
	while(isThreadRunning()) {
        
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
                    ofPixels filteredframe;//, kinectProjImage;
                    filteredframe = framefilter.filter(kinectDepthImage.getPixels());
                    filteredframe.setImageType(OF_IMAGE_GRAYSCALE);
//                    wrldcoord = framefilter.getWrldcoordbuffer();
//                    kinectProjImage = convertProjSpace(filteredframe);
//                    kinectProjImage.setImageType(OF_IMAGE_GRAYSCALE);
                    
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
    }
    kinect.close();
}

