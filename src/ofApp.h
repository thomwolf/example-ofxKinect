#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxCv.h"

#include "ofxUI.h"

#include "ColorMap.h"
#include "FrameFilter.h"
#include "KinectGrabber.h"
#include "vehicle.h"

using namespace cv;

class ofApp : public ofBaseApp{
    
public:
    void setup();
    //   void setupGui();
    void update();
    void draw();
    void drawProj(ofEventArgs & args);
    void exit();
    
    void keyPressed(int key);
    void keyReleased(int key);
    void mouseMoved(int x, int y );
    void mouseDragged(int x, int y, int button);
    void mousePressed(int x, int y, int button);
    void mouseReleased(int x, int y, int button);
    void windowResized(int w, int h);
    void dragEvent(ofDragInfo dragInfo);
    void gotMessage(ofMessage msg);
    
    // gui
    void setupGui();
    ofxUISuperCanvas *          gui;
    ofxUISuperCanvas *		guiImageSettings;
    ofxUISuperCanvas *		guiMappingSettings;
    
    void createVehicles();
    void guiEvent(ofxUIEventArgs &e);
    void guiUpdateLabels();
    //ofxPanel gui;
    shared_ptr<ofAppBaseWindow> projWindow;
    
private:
    
    bool                        enableTestmode, enableCalibration, enableGame;
    
    // calibration settings
    int                         projectorWidth;
    int                         projectorHeight;
    int                         gradFieldresolution;
    
    // UI conf values
    float                   nearclip, farclip;
    float   chessboardSize, chessboardColor,chessboardThreshold, maxReprojError, StabilityTimeInMs;
	int mindepth;
	int maxdepth;
    float contourlinefactor;
    bool horizontalMirror, verticalMirror;

    ofShader                    shader;            //Shader
    ofFbo                       fbo;			//Buffer for intermediate drawing
    ColorMap                    colormap;
    KinectGrabber               kinectgrabber;

    ofxCvGrayscaleImage     FilteredDepthImage;
    ofxCvColorImage         kinectColorImage;
    ofVec2f*                gradientField;
    
    vector<vehicle> vehicles;

    ofParameterGroup labels;
    
    // second window
    //        ofxSecondWindow             secondWindow;
    
};
