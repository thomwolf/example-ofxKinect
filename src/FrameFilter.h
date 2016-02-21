/***********************************************************************
 FrameFilter - Class to filter streams of depth frames arriving from a
 depth camera, with code to detect unstable values in each pixel, and
 fill holes resulting from invalid samples.
 Forked from Oliver Kreylos' Augmented Reality Sandbox (SARndbox).
 ***********************************************************************/

#pragma once
#include "ofMain.h"

class FrameFilter: public ofThread {
public:
	typedef unsigned char RawDepth; // Data type for raw depth values
	typedef float FilteredDepth; // Data type for filtered depth values

    ofThreadChannel<ofPixels> toAnalyze;
    ofThreadChannel<ofPixels> analyzed;

    FrameFilter();
    ~FrameFilter();
    
    bool setup(const unsigned int swidth,const unsigned int sheight,int sNumAveragingSlots, unsigned int newMinNumSamples, unsigned int newMaxVariance, float newHysteresis, bool newSpatialFilter);
    void initiateBuffers(void); // Reinitialise buffers
    void resetBuffers(void);
   void analyze(ofPixels & frame);
    void update();
    bool isFrameNew();
    ofPixels getPixels();
    void draw(float x, float y);
    void draw(float x, float y, float w, float h);
	void setValidDepthInterval(unsigned int newMinDepth,unsigned int newMaxDepth); // Sets the interval of depth values considered by the depth image filter
	void setValidElevationInterval(double newMinElevation,double newMaxElevation); // Sets the interval of elevations relative to the given base plane considered by the depth image filter
	void setStableParameters(unsigned int newMinNumSamples,unsigned int newMaxVariance); // Sets the statistical properties to consider a pixel stable
	void setHysteresis(float newHysteresis); // Sets the stable value hysteresis envelope
	void setRetainValids(bool newRetainValids); // Sets whether the filter retains previous stable values for instable pixels
	void setInstableValue(float newInstableValue); // Sets the depth value to assign to instable pixels
	void setSpatialFilter(bool newSpatialFilter); // Sets the spatial filtering flag
    
private:
    void threadedFunction();
    ofPixels inputframe;
    ofPixels outputframe;
    ofTexture texture;
    bool newFrame;
    
    
    unsigned int width, height; // Width and height of processed frames
    unsigned int inputFrameVersion; // Version number of input frame
	volatile bool runFilterThread; // Flag to keep the background filtering thread running
	float min; // lower bound of valid depth values in depth image space
	float max; // upper bound of valid depth values in depth image space
	int numAveragingSlots; // Number of slots in each pixel's averaging buffer
	RawDepth* averagingBuffer; // Buffer to calculate running averages of each pixel's depth value
	int averagingSlotIndex; // Index of averaging slot in which to store the next frame's depth values
	unsigned int* statBuffer; // Buffer retaining the running means and variances of each pixel's depth value
	unsigned int minNumSamples; // Minimum number of valid samples needed to consider a pixel stable
	unsigned int maxVariance; // Maximum variance to consider a pixel stable
	float hysteresis; // Amount by which a new filtered value has to differ from the current value to update
	bool retainValids; // Flag whether to retain previous stable values if a new pixel in instable, or reset to a default value
	float instableValue; // Value to assign to instable pixels if retainValids is false
	bool spatialFilter; // Flag whether to apply a spatial filter to time-averaged depth values
	RawDepth* validBuffer; // Buffer holding the most recent stable depth value for each pixel
	void* filterThreadMethod(void); // Method for the background filtering thread
	
};
