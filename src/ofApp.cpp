#include "ofApp.h"

using namespace ofxCv;
using namespace cv;

//--------------------------------------------------------------
void ofApp::setup(){
    
    // OF basics
    ofSetFrameRate(60);
    ofBackground(0);
	ofSetVerticalSync(true);
    ofSetLogLevel(OF_LOG_VERBOSE);
    ofSetLogLevel("ofThread", OF_LOG_WARNING);
	
	// settings and defaults
	enableCalibration = false;
	enableTestmode	  = true;
	enableGame = false;
	gotROI = 0;
	kinectROI = ofRectangle(0, 0, 640, 480);
	
	//Mesh calib
	meshwidth = 2; // 640 should be dividable by meshwidth-1
	meshheight = 2; // idem with 480
	
    // contourFinder config
	chessboardThreshold = 60;
	lowThresh = 0.2;
	highThresh = 0.3;
	//	contourFinder.setMinAreaRadius(12);
	//	contourFinder.setThreshold(3);
	//	contourFinder.getTracker().setPersistence(25);
	//	contourFinder.getTracker().setMaximumDistance(150);
	//	contourFinder.setFindHoles(true);
	//	contourFinder.setInvert(false);
	
	// kinect depth clipping
	nearclip = 750;
	farclip = 950;
	int numAveragingSlots=30;
	unsigned int minNumSamples=10;
	unsigned int maxVariance=2;
	float hysteresis=0.5f;
	bool spatialFilter=false;
	gradFieldresolution = 20;
    
    // kinectgrabber: setup
	kinectgrabber.setup();
	//	kinectgrabber.setupClip(nearclip, farclip);
	kinectgrabber.setupFramefilter(numAveragingSlots, minNumSamples, maxVariance, hysteresis, spatialFilter, gradFieldresolution,nearclip, farclip);
	kinectgrabber.startThread();
	
    // calibration config
	projectorWidth = projWindow->getWidth();
	projectorHeight = projWindow->getHeight();
	chessboardSize = 100;
	chessboardColor = 175;
	StabilityTimeInMs = 500;
	maxReprojError = 2.0f;
	chessboardThreshold = 60;
	horizontalMirror = true;
	verticalMirror = true;
	
	// Calibration setup: make the wrapper (to make calibration independant of the drivers...)
    kinectWrapper = new RGBDCamCalibWrapperOfxKinect();
    kinectWrapper->setup(&kinectgrabber.kinect);
    kinectProjectorCalibration.setup(kinectWrapper, projectorWidth, projectorHeight);
	// some default config
    kinectProjectorCalibration.chessboardSize = chessboardSize;
    kinectProjectorCalibration.chessboardColor = chessboardColor;
    kinectProjectorCalibration.setStabilityTimeInMs(StabilityTimeInMs);
    kinectProjectorCalibration.setMirrors(true, true);
	//    maxReprojError = maxReprojError;
    // sets the output
    kinectProjectorOutput.setup(kinectWrapper, projectorWidth, projectorHeight);
    kinectProjectorOutput.setMirrors(false, false);//true, true);
    setupView();
	//kinectProjectorOutput.load("kinectProjector.yml");
	
	// Load colormap
    colormap.load("HeightColorMap.yml");
	
    // prepare shaders and fbo
	contourlinefactor = 50;
    shader.load( "shaderVert.c", "shaderFrag.c" );
    fbo.allocate( projectorWidth, projectorHeight);
	
	FilteredDepthImage.allocate(640, 480);
	FilteredDepthImage.setUseTexture(true);
	
	// setup the gui
    setupGui();
}

//--------------------------------------------------------------
void ofApp::update(){
	
	// Get depth image from kinect grabber
	ofPixels filteredframe;
	if (kinectgrabber.filtered.tryReceive(filteredframe)) {
		///		// If true, `filteredframe` can be used.
		FilteredDepthImage.setFromPixels(filteredframe);
		FilteredDepthImage.updateTexture();
		
		kinectgrabber.lock();
		kinectgrabber.storedframes -= 1;
		//		cout << kinectgrabber.storedframes << endl;
		kinectgrabber.unlock();
		
		thresholdedImage.setFromPixels(FilteredDepthImage.getPixels());
		
		if (enableTestmode){
			fbo.begin(); // drawing colormap texture to shader
			shader.begin();
			shader.setUniformTexture( "texture1", colormap.getTexture(), 1 ); //"1" means that it is texture 1
			shader.setUniform1f("texsize", 255 );
			shader.setUniform1f("contourLineFactor", contourlinefactor);
			ofSetColor( 255, 255, 255 );
			FilteredDepthImage.draw(0, 0);//, projectorWidth, projectorHeight);
			//fbo.draw( 0, 0 );//,projectorWidth, projectorHeight);
			//mesh.draw();
			shader.end();
			fbo.end();
			// find our contours in the label image
			//			if (highThresh != 1.0) cvThreshold(thresholdedImage.getCvImage(), thresholdedImage.getCvImage(), highThresh * 255, 255, CV_THRESH_TOZERO_INV);
			//			if (lowThresh != 0.0) cvThreshold(thresholdedImage.getCvImage(), thresholdedImage.getCvImage(), lowThresh * 255, 255, CV_THRESH_TOZERO);
			//			contourFinder.findContours(thresholdedImage, 12, 640*480, 3, true);
			
			//Create mesh
			//Set up vertices and colors
			mesh.clear();
			
			ofPoint wrld;
			int indx, indy;
			float indz;
			for (int y=0; y<meshheight; y++) {
				for (int x=0; x<meshwidth; x++) {
					indx = 640/(meshwidth-1)*x;
					indy = 480/(meshheight-1)*y;
					indz = farclip;//(255.0-filteredframe.getData()[indy*640+indx])/255.0*(farclip-nearclip)+nearclip;
					wrld = kinectgrabber.kinect.getWorldCoordinateAt(indx, indy, indz);
					mesh.addVertex(wrld);
					//mesh.addColor( ofColor( 0, 0, 0 ) );
					mesh.addTexCoord( ofPoint( indx,indy ) );
				}
			}
			//Set up triangles' indices
			for (int y=0; y<meshheight-1; y++) {
				for (int x=0; x<meshwidth-1; x++) {
					int i1 = x + meshwidth * y;
					int i2 = x+1 + meshwidth * y;
					int i3 = x + meshwidth * (y+1);
					int i4 = x+1 + meshwidth * (y+1);
					mesh.addTriangle( i1, i2, i3 );
					mesh.addTriangle( i2, i4, i3 );
				}
			}
			setNormals( mesh );			//Set normals to the surface
		}
	}
	
	if (enableCalibration) {
		// Get color image from kinect grabber
		ofPixels coloredframe;
		if (kinectgrabber.colored.tryReceive(coloredframe)) {
			///		// If true, `filteredframe` can be used.
			kinectColorImage.setFromPixels(coloredframe);
			
			//		kinectgrabber.lock();
			//		kinectgrabber.storedcoloredframes -= 1;
			//		//		cout << kinectgrabber.storedframes << endl;
			//		kinectgrabber.unlock();
			
			
			// do a very-fast check if chessboard is found
			bool stableBoard = kinectProjectorCalibration.doFastCheck();
			
			//if it is stable, add it.
			if (stableBoard) {
				kinectProjectorCalibration.addCurrentFrame();
				
				// rŽcupe points
				if (gotROI == 1) { // set kinect to max depth range
					kinectgrabber.nearclipchannel.send(500);
					kinectgrabber.farclipchannel.send(4000);
					gotROI = 2;
					
					large = ofPolyline();
					threshold = 220;
				} else if (gotROI == 2) {
					vector<ofVec2f> pointBufFastCheck = kinectProjectorCalibration.getFastCheckResults() ;
					while (threshold < 255){
						thresholdedImage.setFromPixels(FilteredDepthImage.getPixels());
						thresholdedImage.mirror(verticalMirror, horizontalMirror);
						//cvThreshold(thresholdedImage.getCvImage(), thresholdedImage.getCvImage(), highThresh+10, 255, CV_THRESH_TOZERO_INV);
						cvThreshold(thresholdedImage.getCvImage(), thresholdedImage.getCvImage(), threshold, 255, CV_THRESH_TOZERO);
						
						contourFinder.findContours(thresholdedImage, 12, 640*480, 5, true);
						//contourFinder.findContours(thresholdedImage);
						//ofPoint cent = ofPoint(projectorWidth/2, projectorHeight/2);
						
						ofPolyline small = ofPolyline();
						for (int i = 0; i < contourFinder.nBlobs; i++) {
							ofxCvBlob blobContour = contourFinder.blobs[i];
							if (blobContour.hole) {
								//								if(!blobContour.isClosed())
								//									blobContour.close();
								bool ok = true;
								ofPolyline poly = ofPolyline(blobContour.pts);//.getResampledByCount(50);
								for (int j = 0; j < pointBufFastCheck.size(); j++){
									if (!poly.inside(pointBufFastCheck[j].x, pointBufFastCheck[j].y)) {
										ok = false;
										break;
									}
								}
								if (ok) {
									if (small.size() == 0 || poly.getArea() > small.getArea())
										small = poly;
								}
							}
						}
						if (large.getArea() < small.getArea())
							large = small;
						threshold+=1;
					} //else {
					kinectROI = large.getBoundingBox();
					if (horizontalMirror) {
						kinectROI.x = 640 -kinectROI.x;
						kinectROI.width = -kinectROI.width;
					}
					if (verticalMirror) {
						kinectROI.y = 480 -kinectROI.y;
						kinectROI.height = -kinectROI.height;
					}
					kinectROI.standardize();
					// We are finished, set back kinect depth range
					gotROI = 3;
					kinectgrabber.nearclipchannel.send(nearclip);
					kinectgrabber.farclipchannel.send(farclip);
					//}
				}
			}
		}
	}
	
	if (enableGame) {
		if (kinectgrabber.gradient.tryReceive(gradientField)) {
			for (auto & v : vehicles){
				v.applyBehaviours(vehicles, gradientField);
				//			v.borders();
				v.update();
			}
		}
	}
	
    // update the gui labels with the result of our calibraition
    guiUpdateLabels();
}

//--------------------------------------------------------------
void ofApp::draw(){
	ofBackground(0);
	ofSetColor(255);
	
	ofClear(0);
	ofTranslate(320,0);
	if (enableCalibration) {
		ofDrawBitmapString("Kinect Input",0,20);
		kinectColorImage.draw(0,40,320,240);
		FilteredDepthImage.draw(0,20+240+40+40,320,240);
		//	kinectColoredDepth.draw(320+20,40,320,240);
		
		//ofCircle(160, 160, 5); // Kinect center
		
		//if calibrating, then we draw our fast check results here
		
		ofTranslate(0,40);
		vector<ofVec2f> pts = kinectProjectorCalibration.getFastCheckResults();
		for (int i = 0; i < pts.size(); i++) {
			ofSetColor(0,255,0);
			ofFill();
			ofDrawCircle(pts[i].x/2.0, pts[i].y / 2.0, 5);
			ofNoFill();
		}
		ofTranslate(0,-40);
		
		ofSetColor(255);
		
		// draw our calibration gui
		ofDrawBitmapString("Thresholded Kinect Input",320+20,20);
		thresholdedImage.draw(320+20,40,320,240);
		ofTranslate(320+20, 40);
		ofScale(0.5, 0.5);
		large.draw();
		ofScale(2.0, 2.0);
		ofTranslate(-(320+20), -(40));
		//		kinectProjectorCalibration.drawChessboardDebug(320+20,40,320,240);
		//		thresholdedKinect.draw(320+20,40,320,240);
		
		ofDrawBitmapString("Processed Input",0,20+240+20+40);
		kinectProjectorCalibration.drawProcessedInputDebug(0,20+240+40+40,320,240);
		
		ofDrawBitmapString("Reprojected points",320+20,20+240+20+40);
		kinectProjectorCalibration.drawReprojectedPointsDebug(320+20,20+240+40+40,320,240);
		//	} else if (enableTestmode) {
		//		ofDrawBitmapString("Grayscale Image",0,20+240+20+40);
		//		//		kinectDepthImage.draw(0,20+240+40+40,320,240);
		//
		//		ofDrawBitmapString("Contours",320+20,20+240+20+40);
		//
		//		ofTranslate(320+20, 20+240+20+40+20);
		//		ofScale(0.5, 0.5);
		//		//		contourFinder.draw();
		//		ofScale(2.0, 2.0);
		//		ofTranslate(-(320+20), -(20+240+20+40+20));
		//
	} else if (enableTestmode) {
		ofDrawBitmapString("Grayscale Image",0,20+240+20+40);
		FilteredDepthImage.draw(0,20+240+40+40,320,240);
		
		if (gotROI == 3){
			ofTranslate(0,20+240+40+40);
			ofScale(0.5, 0.5);
			ofDrawRectangle(kinectROI);
			ofScale(2.0, 2.0);
			ofTranslate(0, -(20+240+40+40));
		}
		
		ofDrawBitmapString("Contours",320+20,20+240+20+40);
		thresholdedImage.draw(320+20,20+240+20+40+20,320,240);
		ofTranslate(320+20, 20+240+20+40+20);
		ofScale(0.5, 0.5);
		contourFinder.draw();
		ofScale(2.0, 2.0);
		ofTranslate(-(320+20), -(20+240+20+40+20));
	}
	ofTranslate(-320,0);
	ofSetColor(255);
	
    gui->draw();
}
//--------------------------------------------------------------
void ofApp::drawProj(ofEventArgs & args){
    
	//if calibrating, then we draw our fast check results here
	if (enableCalibration) {
		
		// draw the chessboard to our second window
		fbo.begin();		//Start drawing grayscale depth image into buffer
		ofClear(0);
		ofSetColor(255, 170, 170);
		kinectProjectorCalibration.drawChessboard();
		fbo.end();			//End drawing into buffer
		fbo.draw( 0, 0 ,projectorWidth, projectorHeight);
	} else if (enableTestmode) {
					
//		glPushMatrix();
//		glMultMatrixf(matrix);
//		glPushMatrix();
//		fbo.draw(0, 0);
//		glPopMatrix();
//		glPopMatrix();
		
		kinectProjectorOutput.loadCalibratedView();
		fbo.getTexture().bind();
		mesh.draw();
		fbo.getTexture().unbind();
		kinectProjectorOutput.unloadCalibratedView();
			
			//		kinectgrabber.framefilter.displayFlowField();
			//		ofSetColor(255, 190, 70);
			//		ofPoint cent = ofPoint(projectorWidth/2, projectorHeight/2);
			//		for (int i = 0; i < contourFinder.size(); i++) {
			//
			//			ofPolyline blobContour = contourFinder.getPolyline(i);
			//			if(!blobContour.isClosed()){
			//				blobContour.close();
			//			}
			//
			//			//if (!blobContour.inside(cent)) {
			//			ofPolyline rect = blobContour.getResampledByCount(8);
			//				ofBeginShape();
			//			kinectgrabber.lock();
			//				for (int j = 0; j < rect.size() - 1; j++) {
			//					rect[j].z = (farclip-nearclip)*highThresh+nearclip;
			//					ofPoint wrld = kinectgrabber.kinectWrapper->getWorldFromRgbCalibratedXYZ(rect[j], true,true);
			//					ofPoint currVertex = kinectProjectorOutput.projectFromDepthXYZ(rect[j]);
			//					ofVertex(currVertex.x, currVertex.y);
			////					cout << "blob j: "<< j << " rect: "<< rect[j] << " wrld: " << wrld << " currVertex : " << currVertex << endl;
			//				}
			//			kinectgrabber.unlock();
			//				ofEndShape();
			//			//}
			//		}
			
		} else if (enableGame) {
			//1. Drawing into fbo buffer
			fbo.begin();		//Start drawing grayscale depth image into buffer
			ofClear(0);
			ofSetColor(255, 170, 170);
			FilteredDepthImage.ofBaseDraws::draw(0, 0, projectorWidth, projectorHeight);
			fbo.end();			//End drawing into buffer
			
			//		fbo.draw( 0, 0 ,projectorWidth, projectorHeight);
			//2. Drawing to screen through the shader
			shader.begin();
			shader.setUniformTexture( "texture1", colormap.getTexture(), 1 ); //"1" means that it is texture 1
			shader.setUniform1f("texsize", 255 );
			shader.setUniform1f("contourLineFactor", contourlinefactor);
			ofSetColor( 255, 255, 255 );
			fbo.draw( 0, 0 ,projectorWidth, projectorHeight);
			shader.end();
			
			for (auto & v : vehicles){
				v.draw();
			}
			kinectgrabber.framefilter.displayFlowField();
		} else {
			ofBackground(255);
		}
	}
	
	//--------------------------------------------------------------
	void ofApp::exit(){
		
		kinectgrabber.stopThread();
		delete gui;
		delete guiImageSettings;
	}
	
	//--------------------------------------------------------------
	void ofApp::keyPressed(int key){
		
	}
	
	//--------------------------------------------------------------
	void ofApp::keyReleased(int key){
		
	}
	
	//--------------------------------------------------------------
	void ofApp::mouseMoved(int x, int y ){
		
	}
	
	//--------------------------------------------------------------
	void ofApp::mouseDragged(int x, int y, int button){
		
	}
	
	//--------------------------------------------------------------
	void ofApp::mousePressed(int x, int y, int button){
		
	}
	
	//--------------------------------------------------------------
	void ofApp::mouseReleased(int x, int y, int button){
		
	}
	
	//--------------------------------------------------------------
	void ofApp::windowResized(int w, int h){
		
	}
	
	//--------------------------------------------------------------
	void ofApp::gotMessage(ofMessage msg){
		
	}
	
	//--------------------------------------------------------------
	void ofApp::dragEvent(ofDragInfo dragInfo){
		
	}
	
	//--------------------------------------------------------------
	void ofApp::createVehicles() {
		// setup the vehicles
		vehicles.resize(100);
		int screenWidth =projWindow->getWidth();
		int screenHeight =projWindow->getHeight();
		ofPoint location(screenWidth, screenHeight);
		for (auto & v : vehicles){
			v.setup(ofRandom(location.x), ofRandom(location.y), screenWidth, screenHeight);
		}
	}
	
//--------------------------------------------------------------
	void ofApp::setupView() {
		kinectProjectorOutput.load("kinectProjector.yml");
ofPoint des[4];
ofPoint src[]={kinectROI.getTopLeft(), kinectROI.getTopRight(), kinectROI.getBottomRight(),kinectROI.getBottomLeft()};
for (int i = 0; i <4; i++){
	src[i].z = farclip;
	des[i] = kinectProjectorOutput.projectFromDepthXYZ(src[i]);
}
//ofPoint(0,0),ofPoint(image.width,0),ofPoint(image.width,image.height),ofPoint(0,image.height)};

findHomography(src,des,matrix);
	}


//--------------------------------------------------------------
	void ofApp::setupGui() {
		
		float dim = 16;
		float length = 300;
		
		gui = new ofxUISuperCanvas("- ofxKinect example -");
		gui->setColorBack(ofColor(51, 55, 56, 200));
		
		gui->addSpacer(length, 2);
		gui->addWidgetDown(new ofxUIMinimalSlider("Kinect tilt angle", -27, 27, 0.0, length, dim));
		
		gui->addSpacer(length, 2);
		gui->addWidgetDown(new ofxUILabel("Calibration instructions", OFX_UI_FONT_MEDIUM));
		gui->addSpacer(length, 2);
		gui->addWidgetDown(new ofxUILabel("sdf", "1) Move 2nd window to projector", OFX_UI_FONT_SMALL));
		gui->addWidgetDown(new ofxUILabel("sdf", "2) Set it to fullscreen", OFX_UI_FONT_SMALL));
		gui->addWidgetDown(new ofxUILabel("sdf", "3) Activate calibration", OFX_UI_FONT_SMALL));
		gui->addWidgetDown(new ofxUILabel("sdf", "4) Hold flat board ", OFX_UI_FONT_SMALL));
		gui->addWidgetDown(new ofxUILabel("sdf", "5) Keep still during some time to capture", OFX_UI_FONT_SMALL));
		gui->addWidgetDown(new ofxUILabel("sdf", "6) Make 15 captures, then clean the dataset", OFX_UI_FONT_SMALL));
		
		gui->addWidgetDown(new ofxUILabel(" ", OFX_UI_FONT_MEDIUM));
		gui->addWidgetDown(new ofxUILabel("Mode", OFX_UI_FONT_MEDIUM));
		gui->addSpacer(length, 2);
		gui->addWidgetDown(new ofxUIToggle("Activate calibration mode", &enableCalibration, dim, dim));
		gui->addWidgetDown(new ofxUIToggle("Activate game mode", &enableGame, dim, dim));
		
		gui->addWidgetDown(new ofxUILabel(" ", OFX_UI_FONT_MEDIUM));
		gui->addSpacer(length, 2);
		gui->addWidgetDown(new ofxUIFPS(OFX_UI_FONT_MEDIUM));
		
		gui->setPosition(0, 0);//768 - guiImageSettings->getRect()->getHeight());
		gui->autoSizeToFitWidgets();
		
		ofAddListener(gui->newGUIEvent,this,&ofApp::guiEvent);
		
		///////////////////////////////////////////////
		
		guiImageSettings = new ofxUISuperCanvas("Image settings");
		guiImageSettings->setColorBack(ofColor(51, 55, 56, 200));
		
		guiImageSettings->addSpacer(length, 2);
		guiImageSettings->addWidgetDown(new ofxUISlider("Chessboard size", 0, 255, &kinectProjectorCalibration.chessboardSize, length, dim));
		guiImageSettings->addWidgetDown(new ofxUISlider("Chessboard luminosity", 0, 255, &kinectProjectorCalibration.chessboardColor, length, dim));
		guiImageSettings->addWidgetDown(new ofxUISlider("Chessboard detec. threshold", 0, 255, &kinectgrabber.chessboardThreshold, length, dim));
		guiImageSettings->addWidgetDown(new ofxUIToggle("Adaptative threshold", &kinectProjectorCalibration.b_CV_CALIB_CB_ADAPTIVE_THRESH, dim, dim));
		guiImageSettings->addWidgetDown(new ofxUIToggle("Normalize the image", &kinectProjectorCalibration.b_CV_CALIB_CB_NORMALIZE_IMAGE, dim, dim));
		guiImageSettings->addSpacer(length, 2);
		guiImageSettings->addWidgetDown(new ofxUI2DPad("Chessboard position offset", ofVec2f(-1.0, 1.0), ofVec2f(-1.0, 1.0), ofVec2f(0, 0), length, length * 3 / 4));
		
		guiImageSettings->addWidgetDown(new ofxUILabel(" ", OFX_UI_FONT_MEDIUM));
		guiImageSettings->addWidgetDown(new ofxUILabel("Calibration", OFX_UI_FONT_MEDIUM));
		guiImageSettings->addSpacer(length, 2);
		guiImageSettings->addWidgetDown(new ofxUISlider("Max reproj. error", 0.50, 5.00, &maxReprojError, length, dim));
		guiImageSettings->addWidgetDown(new ofxUIButton("Clean dataset (remove > max reproj error)", false, dim, dim));
		guiImageSettings->addWidgetDown(new ofxUIButton("Clear dataset (remove all)", false, dim, dim));
		guiImageSettings->addWidgetDown(new ofxUILabel(" ", OFX_UI_FONT_LARGE));
		guiImageSettings->addWidgetDown(new ofxUILabel("errorLabel", "Avg Reprojection error: 0.0", OFX_UI_FONT_SMALL));
		guiImageSettings->addWidgetDown(new ofxUILabel("capturesLabel", "Number of captures: 0", OFX_UI_FONT_SMALL));
		
		guiImageSettings->addWidgetDown(new ofxUIToggle("Activate test mode", &enableTestmode, dim, dim));
		guiImageSettings->addWidgetDown(new ofxUIToggle("Horizontal mirror", &horizontalMirror, dim, dim));
		guiImageSettings->addWidgetDown(new ofxUIToggle("Vertical Mirror", &verticalMirror, dim, dim));
		//	guiImageSettings->addSpacer(length, 2);
		//	guiImageSettings->addWidgetDown(new ofxUILabel("lbl", "Tracking options", OFX_UI_FONT_MEDIUM));
		//	guiImageSettings->addWidgetDown(new ofxUIRangeSlider("Thresholds", 0.0, 1.0, &kinectgrabber.lowThresh, &kinectgrabber.highThresh, length, dim));
		
		guiImageSettings->autoSizeToFitWidgets();
		guiImageSettings->setPosition(0, 0);//768 - guiImageSettings->getRect()->getHeight());
		//guiImageSettings->toggleMinified();
		guiImageSettings->setVisible(enableCalibration);
		ofAddListener(guiImageSettings->newGUIEvent,this,&ofApp::guiEvent);
		
		///////////////////////////////////////////////
		
		guiMappingSettings = new ofxUISuperCanvas("Mapping settings");
		guiMappingSettings->setColorBack(ofColor(51, 55, 56, 200));
		
		guiMappingSettings->addSpacer(length, 2);
		guiMappingSettings->addWidgetDown(new ofxUIRangeSlider("Kinect range", 500.0, 1500.0, &nearclip, &farclip, length, dim));
		
		guiMappingSettings->addSpacer(length, 2);
		guiMappingSettings->addWidgetDown(new ofxUISlider("Contourline factor", 0.0, 255, &contourlinefactor, length, dim));
		guiMappingSettings->addSpacer(length, 2);
		guiMappingSettings->addWidgetDown(new ofxUIRangeSlider("Threshold", 0.0, 1.0, &lowThresh, &highThresh, length, dim));
		
		guiMappingSettings->autoSizeToFitWidgets();
		guiMappingSettings->setPosition(1280 - guiMappingSettings->getRect()->getWidth(), 0);
		//guiImageSettings->toggleMinified();
		ofAddListener(guiMappingSettings->newGUIEvent,this,&ofApp::guiEvent);
		
	}
	
	//--------------------------------------------------------------
	void ofApp::guiUpdateLabels() {
		ofxUILabel* l;
		l = (ofxUILabel*) guiImageSettings->getWidget("errorLabel");
		l->setLabel("Avg Reprojection error: " + ofToString(kinectProjectorCalibration.getReprojectionError(), 2));
		//l->setLabel("MaxDepth: " + ofToString(maxdepth));
		
		l = (ofxUILabel*) guiImageSettings->getWidget("capturesLabel");
		l->setLabel("Number of captures: " + ofToString(kinectProjectorCalibration.getDatabaseSize()));
		//l->setLabel("MinDepth: " + ofToString(mindepth));
	}
	
	//--------------------------------------------------------------
	void ofApp::guiEvent(ofxUIEventArgs &e) {
		string name = e.widget->getName();
		int kind = e.widget->getKind();
		if (name == "Clean dataset (remove > max reproj error)") {
			ofxUIButton* b = (ofxUIButton*)e.widget;
			if(b->getValue()) kinectProjectorCalibration.clean(maxReprojError);
		} else if (name == "Clear dataset (remove all)") {
			ofxUIButton* b = (ofxUIButton*)e.widget;
			if(b->getValue()) kinectProjectorCalibration.clearAll();
		}else if (name == "Activate game mode") {
			ofxUIButton* b = (ofxUIButton*)e.widget;
			if(b->getValue()) {
				createVehicles();
				enableTestmode = false;
				enableCalibration = false;
				enableGame = true;
			}
		} else if (name == "Activate test mode") {
			ofxUIButton* b = (ofxUIButton*)e.widget;
			if(b->getValue()) {
				enableTestmode = true;
				enableCalibration = false;
				enableGame = false;
				kinectgrabber.lock();
				kinectgrabber.setTestmode();
				kinectgrabber.unlock();
				setupView();
				guiImageSettings->setVisible(false);
				guiMappingSettings->setVisible(true);
				//			gui->setVisible(true);
			}
		} else if (name == "Activate calibration mode") {
			ofxUIButton* b = (ofxUIButton*)e.widget;
			if(b->getValue()){
				enableCalibration = true;
				enableTestmode = false;
				enableGame = false;
				gotROI = 1;
				kinectgrabber.lock();
				kinectgrabber.setCalibrationmode();
				kinectgrabber.unlock();
				guiImageSettings->setVisible(true);
				guiMappingSettings->setVisible(false);
				//			gui->toggleVisible();
			}
		} else if (name == "Kinect tilt angle") {
			//		ofxUISlider *slider = (ofxUISlider *) e.widget;
			//		kinect.setCameraTiltAngle(slider->getValue());
		} else if (name == "Chessboard position offset") {
			ofxUI2DPad *pad = (ofxUI2DPad *) e.widget;
			ofVec2f trans = pad->getValue();
			kinectProjectorCalibration.setChessboardTranslation((trans.x * projectorWidth) - projectorWidth / 2, (trans.y * projectorHeight) - projectorHeight / 2);
		} else if (name == "Kinect range") {
			kinectgrabber.nearclipchannel.send(nearclip);
			kinectgrabber.farclipchannel.send(farclip);
		} else if (name == "Horizontal mirror" || name == "Vertical mirror") {
			kinectProjectorCalibration.setMirrors(horizontalMirror, verticalMirror);
			//		kinectProjectorOutput.setMirrors(horizontalMirror, verticalMirror);
		}
	}

void ofApp::findHomography(ofPoint src[4], ofPoint dst[4], float homography[16]){
	// arturo castro - 08/01/2010
	//
	// create the equation system to be solved
	//
	// from: Multiple View Geometry in Computer Vision 2ed
	//       Hartley R. and Zisserman A.
	//
	// x' = xH
	// where H is the homography: a 3 by 3 matrix
	// that transformed to inhomogeneous coordinates for each point
	// gives the following equations for each point:
	//
	// x' * (h31*x + h32*y + h33) = h11*x + h12*y + h13
	// y' * (h31*x + h32*y + h33) = h21*x + h22*y + h23
	//
	// as the homography is scale independent we can let h33 be 1 (indeed any of the terms)
	// so for 4 points we have 8 equations for 8 terms to solve: h11 - h32
	// after ordering the terms it gives the following matrix
	// that can be solved with gaussian elimination:
	
	float P[8][9]={
		{-src[0].x, -src[0].y, -1,   0,   0,  0, src[0].x*dst[0].x, src[0].y*dst[0].x, -dst[0].x }, // h11
		{  0,   0,  0, -src[0].x, -src[0].y, -1, src[0].x*dst[0].y, src[0].y*dst[0].y, -dst[0].y }, // h12
		
		{-src[1].x, -src[1].y, -1,   0,   0,  0, src[1].x*dst[1].x, src[1].y*dst[1].x, -dst[1].x }, // h13
		{  0,   0,  0, -src[1].x, -src[1].y, -1, src[1].x*dst[1].y, src[1].y*dst[1].y, -dst[1].y }, // h21
		
		{-src[2].x, -src[2].y, -1,   0,   0,  0, src[2].x*dst[2].x, src[2].y*dst[2].x, -dst[2].x }, // h22
		{  0,   0,  0, -src[2].x, -src[2].y, -1, src[2].x*dst[2].y, src[2].y*dst[2].y, -dst[2].y }, // h23
		
		{-src[3].x, -src[3].y, -1,   0,   0,  0, src[3].x*dst[3].x, src[3].y*dst[3].x, -dst[3].x }, // h31
		{  0,   0,  0, -src[3].x, -src[3].y, -1, src[3].x*dst[3].y, src[3].y*dst[3].y, -dst[3].y }, // h32
	};
	
	gaussian_elimination(&P[0][0],9);
	
	// gaussian elimination gives the results of the equation system
	// in the last column of the original matrix.
	// opengl needs the transposed 4x4 matrix:
	float aux_H[]={ P[0][8],P[3][8],0,P[6][8], // h11  h21 0 h31
		P[1][8],P[4][8],0,P[7][8], // h12  h22 0 h32
		0      ,      0,0,0,       // 0    0   0 0
		P[2][8],P[5][8],0,1};      // h13  h23 0 h33
	
	for(int i=0;i<16;i++) homography[i] = aux_H[i];
}

void ofApp::gaussian_elimination(float *input, int n){
	// arturo castro - 08/01/2010
	//
	// ported to c from pseudocode in
	// [http://en.wikipedia.org/wiki/Gaussian-elimination](http://en.wikipedia.org/wiki/Gaussian-elimination)
	
	float * A = input;
	int i = 0;
	int j = 0;
	int m = n-1;
	while (i < m && j < n){
		// Find pivot in column j, starting in row i:
		int maxi = i;
		for(int k = i+1; k<m; k++){
			if(fabs(A[k*n+j]) > fabs(A[maxi*n+j])){
				maxi = k;
			}
		}
		if (A[maxi*n+j] != 0){
			//swap rows i and maxi, but do not change the value of i
			if(i!=maxi)
				for(int k=0;k<n;k++){
					float aux = A[i*n+k];
					A[i*n+k]=A[maxi*n+k];
					A[maxi*n+k]=aux;
				}
			//Now A[i,j] will contain the old value of A[maxi,j].
			//divide each entry in row i by A[i,j]
			float A_ij=A[i*n+j];
			for(int k=0;k<n;k++){
				A[i*n+k]/=A_ij;
			}
			//Now A[i,j] will have the value 1.
			for(int u = i+1; u< m; u++){
				//subtract A[u,j] * row i from row u
				float A_uj = A[u*n+j];
				for(int k=0;k<n;k++){
					A[u*n+k]-=A_uj*A[i*n+k];
				}
				//Now A[u,j] will be 0, since A[u,j] - A[i,j] * A[u,j] = A[u,j] - 1 * A[u,j] = 0.
			}
			
			i++;
		}
		j++;
	}
	
	//back substitution
	for(int i=m-2;i>=0;i--){
		for(int j=i+1;j<n-1;j++){
			A[i*n+m]-=A[i*n+j]*A[j*n+m];
			//A[i*n+j]=0;
		}  
	}  
}
//--------------------------------------------------------------
//Universal function which sets normals for the triangle mesh
void ofApp::setNormals( ofMesh &mesh ){
	
	//The number of the vertices
	int nV = mesh.getNumVertices();
	
	//The number of the triangles
	int nT = mesh.getNumIndices() / 3;
	
	vector<ofPoint> norm( nV );			//Array for the normals
	
	//Scan all the triangles. For each triangle add its
	//normal to norm's vectors of triangle's vertices
	for (int t=0; t<nT; t++) {
		
		//Get indices of the triangle t
		int i1 = mesh.getIndex( 3 * t );
		int i2 = mesh.getIndex( 3 * t + 1 );
		int i3 = mesh.getIndex( 3 * t + 2 );
		
		//Get vertices of the triangle
		const ofPoint &v1 = mesh.getVertex( i1 );
		const ofPoint &v2 = mesh.getVertex( i2 );
		const ofPoint &v3 = mesh.getVertex( i3 );
		
		//Compute the triangle's normal
		ofPoint dir = ( (v2 - v1).crossed( v3 - v1 ) ).normalized();
		
		//Accumulate it to norm array for i1, i2, i3
		norm[ i1 ] += dir;
		norm[ i2 ] += dir;
		norm[ i3 ] += dir;
	}
	
	//Normalize the normal's length
	for (int i=0; i<nV; i++) {
		norm[i].normalize();
	}
	
	//Set the normals to mesh
	mesh.clearNormals();
	mesh.addNormals( norm );
}

