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
	
	// kinect depth clipping
	nearclip = 500;
	farclip = 4000;
    // framefilter: configure backend
	int numAveragingSlots=20;
	unsigned int minNumSamples=10;
	unsigned int maxVariance=2;
	float hysteresis=0.1f;
	bool spatialFilter=true;
    
    // calibration config
	projectorWidth = 640;
	projectorHeight = 480;
	chessboardSize = 100;
	chessboardColor = 175;
	StabilityTimeInMs = 500;
	maxReprojError = 2.0f;
	chessboardThreshold = 60;
	gradFieldresolution = 10;
	horizontalMirror = false;
	verticalMirror = false;
	
    // kinectgrabber: setup
	kinectgrabber.setup();
	kinectgrabber.setupFramefilter(numAveragingSlots, minNumSamples, maxVariance, hysteresis, spatialFilter, gradFieldresolution);
	kinectgrabber.setupClip(nearclip, farclip);
	kinectgrabber.setupCalibration(projectorWidth, projectorHeight, chessboardSize, chessboardColor, StabilityTimeInMs, maxReprojError);
	kinectgrabber.startThread();
	
    // Load colormap
    colormap.load("HeightColorMap.yml");
	
    // prepare shaders and fbo
	contourlinefactor = 50;
    shader.load( "shaderVert.c", "shaderFrag.c" );
    fbo.allocate( projectorWidth, projectorHeight);
	
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
		
		kinectgrabber.lock();
		kinectgrabber.storedframes -= 1;
		//		cout << kinectgrabber.storedframes << endl;
		kinectgrabber.unlock();
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
			bool stableBoard = kinectgrabber.kinectProjectorCalibration.doFastCheck();
			
			//if it is stable, add it.
			if (stableBoard) {
				kinectgrabber.kinectProjectorCalibration.addCurrentFrame();
			}
		}
	}
	
	if (enableGame) {
		for (auto & v : vehicles){
			v.applyBehaviours(vehicles);
			v.borders();
			v.update();
		}
	}

    // update the gui labels with the result of our calibraition
    guiUpdateLabels();
}

//--------------------------------------------------------------
void ofApp::draw(){
	ofBackground(0);
	ofSetColor(255);
	
	if (enableCalibration) {
		ofTranslate(320,0);
		ofDrawBitmapString("Kinect Input",0,20);
		kinectColorImage.draw(0,40,320,240);
		FilteredDepthImage.draw(0,20+240+40+40,320,240);
		//	kinectColoredDepth.draw(320+20,40,320,240);
		
		//ofCircle(160, 160, 5); // Kinect center
		
		//if calibrating, then we draw our fast check results here
		
		ofTranslate(0,40);
		vector<ofVec2f> pts = kinectgrabber.kinectProjectorCalibration.getFastCheckResults();
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
		kinectgrabber.kinectProjectorCalibration.drawChessboardDebug(320+20,40,320,240);
		//		thresholdedKinect.draw(320+20,40,320,240);
		
		ofDrawBitmapString("Processed Input",0,20+240+20+40);
		kinectgrabber.kinectProjectorCalibration.drawProcessedInputDebug(0,20+240+40+40,320,240);
		
		ofDrawBitmapString("Reprojected points",320+20,20+240+20+40);
		kinectgrabber.kinectProjectorCalibration.drawReprojectedPointsDebug(320+20,20+240+40+40,320,240);
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
		ofTranslate(-320,0);
		ofSetColor(255);
	}
	
    gui->draw();
}
//--------------------------------------------------------------
void ofApp::drawProj(ofEventArgs & args){
    
	//if calibrating, then we draw our fast check results here
	if (enableCalibration) {
		
		// draw the chessboard to our second window
		ofClear(0);
		kinectgrabber.kinectProjectorCalibration.drawChessboard();
	} else if (enableTestmode) {
		
		//1. Drawing into fbo buffer
		fbo.begin();		//Start drawing grayscale depth image into buffer
		ofClear(0);
		ofSetColor(255, 170, 170);
		FilteredDepthImage.ofBaseDraws::draw(0, 0, projectorWidth, projectorHeight);
		fbo.end();			//End drawing into buffer
		
		fbo.draw( 0, 0 );
		//2. Drawing to screen through the shader
		shader.begin();
		shader.setUniformTexture( "texture1", colormap.getTexture(), 1 ); //"1" means that it is texture 1
		shader.setUniform1f("texsize", 255 );
		shader.setUniform1f("contourLineFactor", contourlinefactor);
		ofSetColor( 255, 255, 255 );
		fbo.draw( 0, 0 );
		shader.end();
		
	} else if (enableGame) {
		//1. Drawing into fbo buffer
		fbo.begin();		//Start drawing grayscale depth image into buffer
		ofClear(0);
		ofSetColor(255, 170, 170);
		FilteredDepthImage.ofBaseDraws::draw(0, 0, projectorWidth, projectorHeight);
		fbo.end();			//End drawing into buffer
		
		fbo.draw( 0, 0 );
		//2. Drawing to screen through the shader
		shader.begin();
		shader.setUniformTexture( "texture1", colormap.getTexture(), 1 ); //"1" means that it is texture 1
		shader.setUniform1f("texsize", 255 );
		shader.setUniform1f("contourLineFactor", contourlinefactor);
		ofSetColor( 255, 255, 255 );
		fbo.draw( 0, 0 );
		shader.end();
		
		for (auto & v : vehicles){
			v.draw();
		}
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
	
    gui->autoSizeToFitWidgets();
    
    ofAddListener(gui->newGUIEvent,this,&ofApp::guiEvent);
	
	///////////////////////////////////////////////
	
	guiImageSettings = new ofxUISuperCanvas("Image settings");
	guiImageSettings->setColorBack(ofColor(51, 55, 56, 200));
	
	guiImageSettings->addSpacer(length, 2);
	guiImageSettings->addWidgetDown(new ofxUISlider("Chessboard size", 0, 255, &kinectgrabber.kinectProjectorCalibration.chessboardSize, length, dim));
	guiImageSettings->addWidgetDown(new ofxUISlider("Chessboard luminosity", 0, 255, &kinectgrabber.kinectProjectorCalibration.chessboardColor, length, dim));
	guiImageSettings->addWidgetDown(new ofxUISlider("Chessboard detec. threshold", 0, 255, &kinectgrabber.chessboardThreshold, length, dim));
	guiImageSettings->addWidgetDown(new ofxUIToggle("Adaptative threshold", &kinectgrabber.kinectProjectorCalibration.b_CV_CALIB_CB_ADAPTIVE_THRESH, dim, dim));
	guiImageSettings->addWidgetDown(new ofxUIToggle("Normalize the image", &kinectgrabber.kinectProjectorCalibration.b_CV_CALIB_CB_NORMALIZE_IMAGE, dim, dim));
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
	guiMappingSettings->addWidgetDown(new ofxUIRangeSlider("Kinect range", 500.0, 4000.0, &nearclip, &farclip, length, dim));
	
	guiMappingSettings->addSpacer(length, 2);
	guiMappingSettings->addWidgetDown(new ofxUISlider("Contourline factor", 0.0, 255, &contourlinefactor, length, dim));
	
	guiMappingSettings->autoSizeToFitWidgets();
	guiMappingSettings->setPosition(640 - guiMappingSettings->getRect()->getWidth(), 0);
	//guiImageSettings->toggleMinified();
	ofAddListener(guiMappingSettings->newGUIEvent,this,&ofApp::guiEvent);
    
}

//--------------------------------------------------------------
void ofApp::guiUpdateLabels() {
    ofxUILabel* l;
    l = (ofxUILabel*) guiImageSettings->getWidget("errorLabel");
	l->setLabel("Avg Reprojection error: " + ofToString(kinectgrabber.kinectProjectorCalibration.getReprojectionError(), 2));
    //l->setLabel("MaxDepth: " + ofToString(maxdepth));
    
    l = (ofxUILabel*) guiImageSettings->getWidget("capturesLabel");
	l->setLabel("Number of captures: " + ofToString(kinectgrabber.kinectProjectorCalibration.getDatabaseSize()));
    //l->setLabel("MinDepth: " + ofToString(mindepth));
}

//--------------------------------------------------------------
void ofApp::guiEvent(ofxUIEventArgs &e) {
	string name = e.widget->getName();
	int kind = e.widget->getKind();
	if (name == "Clean dataset (remove > max reproj error)") {
		ofxUIButton* b = (ofxUIButton*)e.widget;
		if(b->getValue()) kinectgrabber.kinectProjectorCalibration.clean(maxReprojError);
	} else if (name == "Clear dataset (remove all)") {
		ofxUIButton* b = (ofxUIButton*)e.widget;
		if(b->getValue()) kinectgrabber.kinectProjectorCalibration.clearAll();
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
		kinectgrabber.kinectProjectorCalibration.setChessboardTranslation((trans.x * projectorWidth) - projectorWidth / 2, (trans.y * projectorHeight) - projectorHeight / 2);
	} else if (name == "Kinect range") {
		kinectgrabber.nearclipchannel.send(nearclip);
		kinectgrabber.farclipchannel.send(farclip);
	} else if (name == "Horizontal mirror" || name == "Vertical mirror") {
		kinectgrabber.kinectProjectorCalibration.setMirrors(horizontalMirror, verticalMirror);
	}
}
