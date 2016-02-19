/***********************************************************************
 ColorMap - Class to map from scalar values to RGBA colors.
 Inspired by Oliver Kreylos Vrui Colormap file
 which is part of the OpenGL Support Library (GLSupport).
 ***********************************************************************/

#ifndef ColorMap_INCLUDED
#define ColorMap_INCLUDED

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxOpenCv.h"
/*#include <GL/gl.h>
 #include <GL/GLColor.h>
 */
using namespace ofxCv;
using namespace cv;
class ColorMap
{
	/* Embedded classes: */
public:
	typedef ofColor Color; // Type of color entries
	
	enum CreationTypes // Types for automatic palette generation
    {
		GREYSCALE=0x1,RAINBOW=0x2,
		CONSTANT_ALPHA=0x4,RAMP_ALPHA=0x8
    };
	
	/* Elements: */
private:
	int numEntries; // Number of colors in the map
    ofPixels entries; // Array of RGBA entries
    ofImage tex;
    
	double min,max; // The scalar value range
	double factor,offset; // The scaling factors to map data values to indices
	
	/* Private methods: */
	void setNumEntries(int newNumEntries); // Changes the color map's size
	void copyMap(int newNumEntries,const Color* newEntries,double newMin,double newMax); // Copies from another color map
	
	/* Constructors and destructors: */
public:
	~ColorMap(void);
	
	/* Methods: */
	bool load(string path, bool absolute = false); // Loads a color map from a file
	ColorMap& setColors(int newNumEntries,const Color* newEntries); // Sets the color map array directly
    bool createFile(string filename, bool absolute); //create a colormap file
    
    //	void save(const char* fileName) const; // Saves a 256-entry color map to a file
	double getScalarRangeMin(void) const // Returns minimum of scalar value range
    {
		return min;
    }
	double getScalarRangeMax(void) const // Returns maximum of scalar value range
    {
		return max;
    }
	ColorMap& setScalarRange(double newMin,double newMax); // Changes the scalar value range
	ColorMap& changeTransparency(float gamma); // Applies a gamma function to the transparency values
	ColorMap& premultiplyAlpha(void); // Converts the colors into premultiplied alpha format for easier compositing
	int getNumEntries(void) const // Returns the number of entries in the map
    {
		return numEntries;
    }
	Color operator()(int scalar) const; // Returns the color for a scalar value using linear interpolation
    ofImage getTexture(); // return color map
    
};

#endif
