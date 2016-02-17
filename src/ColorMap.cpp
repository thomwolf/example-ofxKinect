/***********************************************************************
 ColorMap - Class to map from scalar values to RGBA colors.
 Inspired by Oliver Kreylos Vrui Colormap file
 which is part of the OpenGL Support Library (GLSupport).
 ***********************************************************************/

/*#include <math.h>
#include <string.h>
#include <Misc/ThrowStdErr.h>
#include <Misc/Endianness.h>
#include <IO/File.h>
#include <IO/OpenFile.h>
*/
#include <ColorMap.h>
using namespace ofxCv;
using namespace cv;

/****************
Helper functions:
****************/

/*namespace Misc {

template <>
inline
void
swapEndianness(
	ofColor& color)
	{
	swapEndianness(color.getRgba(),4);
	}

template <>
inline
void
swapEndianness(
	ofColor* colors,
	size_t numColors)
	{
	for(size_t i=0;i<numColors;++i)
		swapEndianness(colors[i].getRgba(),4);
	}

}*/

/***************************
Methods of class ColorMap:
***************************/

void ColorMap::setNumEntries(int newNumEntries)
	{
	/* Check if number actually changed: */
	if(numEntries!=newNumEntries)
		{
		/* Reallocate entry array: */
		delete[] entries;
		numEntries=newNumEntries;
		entries=new Color[numEntries];
		
		/* Recalculate mapping factors: */
		factor=double(numEntries-1)/(max-min);
		offset=min*factor;
		}
	}

void ColorMap::copyMap(int newNumEntries,const ColorMap::Color* newEntries,double newMin,double newMax)
	{
	/* Set mapping range: */
	min=newMin;
	max=newMax;
	
	/* Set number of entries: */
	setNumEntries(newNumEntries);
	
	/* Copy map entries: */
	memcpy(entries,newEntries,numEntries*sizeof(Color));
	}

ColorMap::ColorMap(GLenum type,float alphaMax,float alphaGamma,double sMin,double sMax)
	:numEntries(0),entries(0),min(sMin),max(sMax)
	{
	/* Create entry array: */
	setNumEntries(256);
	
	/* Create the palette colors: */
	GLenum colorType=type&(GREYSCALE|RAINBOW);
	if(colorType==GREYSCALE)
		{
		for(int i=0;i<256;++i)
			{
			entries[i][0]=float(i)/255.0f;
			entries[i][1]=float(i)/255.0f;
			entries[i][2]=float(i)/255.0f;
			}
		}
	else if(colorType==RAINBOW)
		{
		for(int i=0;i<256;++i)
			{
			/* Create rainbow colors: */
			double rad=double(i)*(2.0*M_PI/256.0);
			if(rad<=2.0*M_PI/3.0)
				entries[i][0]=cos(0.75*rad);
			else if(rad>=4.0*M_PI/3.0)
				entries[i][0]=cos(0.75*(2.0*M_PI-rad));
			else
				entries[i][0]=0.0;
			entries[i][1]=sin(0.75*rad);
			if(entries[i][1]<0.0)
				entries[i][1]=0.0;
			entries[i][2]=sin(0.75*(rad-2.0*M_PI/3.0));
			if(entries[i][2]<0.0)
				entries[i][2]=0.0;
			}
		}
	
	/* Create the palette opacities: */
	GLenum alphaType=type&(CONSTANT_ALPHA|RAMP_ALPHA);
	if(alphaType==CONSTANT_ALPHA)
		{
		for(int i=0;i<256;++i)
			entries[i][3]=alphaMax;
		}
	else if(alphaType==RAMP_ALPHA)
		{
		double ag=double(alphaGamma);
		for(int i=0;i<256;++i)
			entries[i][3]=alphaMax*pow(double(i)/255.0,ag);
		}
	}

ColorMap::ColorMap(int sNumEntries,const ColorMap::Color* sEntries,double sMin,double sMax)
	:numEntries(0),entries(0),min(sMin),max(sMax)
	{
	/* Copy entry array: */
	copyMap(sNumEntries,sEntries,sMin,sMax);
	}

/*ColorMap::ColorMap(int numKeys,const Color* colors,const double* keys,int sNumEntries)
	:numEntries(0),entries(0),min(keys[0]),max(keys[numKeys-1])
	{
	}*/

/*ColorMap::ColorMap(const char* fileName,double sMin,double sMax)
	:numEntries(0),entries(0),min(sMin),max(sMax)
	{
	// Load color map file: 
	load(fileName);
	}*/

ColorMap::ColorMap(const ColorMap& source)
	:numEntries(0),entries(0)
	{
	/* Copy entry array: */
	copyMap(source.numEntries,source.entries,source.min,source.max);
	}

ColorMap& ColorMap::operator=(const ColorMap& source)
	{
	if(this!=&source)
		{
		/* Copy entry array: */
		copyMap(source.numEntries,source.entries,source.min,source.max);
		}
	
	return *this;
	}

ColorMap::~ColorMap(void)
	{
	delete[] entries;
	}

bool ColorMap::load(string filename, bool absolute) {
 	std::vector<ofColor> heightMapColors;
	std::vector<double> heightMapKeys;
    int numKeys;
/*    heightMapColors.push_back( ofColor(0, 0,  80));
heightMapColors.push_back( ofColor(0,  30, 100));
heightMapColors.push_back( ofColor(0,  50, 102));
heightMapColors.push_back( ofColor(19, 108, 160));
heightMapColors.push_back( ofColor(24, 140, 205));
heightMapColors.push_back( ofColor(135, 206, 250));
heightMapColors.push_back( ofColor(176, 226, 255));
heightMapColors.push_back( ofColor(0,  97,  71));
heightMapColors.push_back( ofColor(16, 122,  47));
heightMapColors.push_back( ofColor(232, 215, 125));
heightMapColors.push_back( ofColor(161,  67,   0));
heightMapColors.push_back( ofColor(130,  30,  30));
heightMapColors.push_back( ofColor(161, 161, 161));
heightMapColors.push_back( ofColor(206, 206, 206));
heightMapColors.push_back( ofColor(255, 255, 255));

    heightMapKeys.push_back(-40.0);
    heightMapKeys.push_back(-30.0);
    heightMapKeys.push_back(-20.0);
    heightMapKeys.push_back(-12.5);
    heightMapKeys.push_back(-0.75);
    heightMapKeys.push_back(-0.25);
    heightMapKeys.push_back(-0.05);
    heightMapKeys.push_back(0.0);
    heightMapKeys.push_back(0.25);
    heightMapKeys.push_back(2.5);
    heightMapKeys.push_back(6);
    heightMapKeys.push_back(9);
    heightMapKeys.push_back(14);
    heightMapKeys.push_back(20);
    heightMapKeys.push_back(25);
    
    
    FileStorage fs(ofToDataPath(filename, absolute), FileStorage::WRITE);
    fs << "ColorMap" << "[";
    for( int i = 0; i < heightMapColors.size(); i++ )
    {
        fs << "{:" << "z" << heightMapKeys[i] << "color" << heightMapColors[i].getHex() << "}";
    }
    fs << "]";*/
    
    FileStorage fs(ofToDataPath(filename, absolute), FileStorage::READ);
    FileNode features = fs["ColorMap"];
    FileNodeIterator it = features.begin(), it_end = features.end();
    int idx = 0;
    std::vector<uchar> lbpval;
    
    // iterate through a sequence using FileNodeIterator
    for( ; it != it_end; ++it, idx++ )
    {
        cout << "color #" << idx << ": ";
        cout << "z=" << (double)(*it)["z"] << ", color=" << (int)(*it)["color"] << endl;
        heightMapKeys.push_back((double)(*it)["z"]);
        heightMapColors.push_back(ofColor::fromHex((int)(*it)["color"]));
   }
    fs.release();
//	heightMap=ColorMap(heightMapKeys.size(),&heightMapColors[0],&heightMapKeys[0]);
    /*ColorMap::ColorMap(int numKeys,const Color* colors,const double* keys,int sNumEntries)
                        :numEntries(0),entries(0),min(keys[0]),max(keys[numKeys-1])*/
	
    /* Create entry array: */
	numKeys = heightMapKeys.size();
    int sNumEntries =256;
    setNumEntries(sNumEntries);
	/* Evaluate the color function: */
	for(int i=0;i<numKeys;++i)
    {
		/* Calculate the key value for this color map entry: */
		double val=double(i)*(heightMapKeys[numKeys-1]-heightMapKeys[0])/double(numKeys-1)+heightMapKeys[0];
		
		/* Find the piecewise linear segment of the color function containing the key value using binary search: */
		int l=0;
		int r=numKeys;
		while(r-l>1)
        {
			/* Enforce the invariant keys[l]<=val<keys[r]: */
			int m=(l+r)>>1;
			if(heightMapKeys[m]<=val)
				l=m;
			else
				r=m;
        }
		
		/* Evaluate the linear segment: */
		if(r<numEntries)
        {
			/* Interpolate linearly: */
			float w=float((val-heightMapKeys[l])/(heightMapKeys[r]-heightMapKeys[l]));
            entries[i]=heightMapColors[l]*(1.0f-w)+heightMapColors[r]*w;
        }
		else
        {
			/* There is nothing to the right of the last key, so no need to interpolate: */
			entries[i]=heightMapColors[numKeys-1];
        }
    }
	return true;
}

ColorMap& ColorMap::setColors(int newNumEntries,const ColorMap::Color* newEntries)
	{
	/* Copy entry array: */
	copyMap(newNumEntries,newEntries,min,max);
	
	return *this;
	}

/*void ColorMap::save(const char* fileName) const
	{
	// We only save 256-entry maps!
	if(numEntries!=256)
		Misc::throwStdErr("ColorMap::save: Attempt to save color map with wrong number of entries");
	
	// Write color entries to file:
	IO::FilePtr file(IO::openFile(fileName,IO::File::WriteOnly));
	file->setEndianness(Misc::BigEndian);
	file->write(entries,numEntries);
	}*/

ColorMap& ColorMap::setScalarRange(double newMin,double newMax)
	{
	min=newMin;
	max=newMax;
	factor=double(numEntries-1)/(max-min);
	offset=min*factor;
	
	return *this;
	}

ColorMap& ColorMap::changeTransparency(float gamma)
	{
	/* Change the transparencies (not opacities!): */
	double dg=double(gamma);
	for(int i=0;i<numEntries;++i)
		entries[i][3]=float(1.0-pow(1.0-double(entries[i][3]),dg));
	
	return *this;
	}

ColorMap& ColorMap::premultiplyAlpha(void)
	{
	/* Premultiply each entry: */
	for(int i=0;i<numEntries;++i)
		for(int j=0;j<3;++j)
			entries[i][j]*=entries[i][3];
	
	return *this;
	}

ColorMap::Color ColorMap::operator()(double scalar) const
	{
	/* Check for out-of-bounds arguments: */
	if(scalar<=min)
		return entries[0];
	else if(scalar>=max)
		return entries[numEntries-1];
	
	/* Calculate the base map index: */
	scalar=scalar*factor-offset;
	int index=int(floor(scalar));
	if(index==numEntries-1)
		--index;
	scalar-=double(index);
	Color result;
    result=entries[index]*(1.0-scalar)+entries[index+1]*scalar;
	
	return result;
	}

ofxCvColorImage ColorMap::operator()(ofxCvGrayscaleImage input) const // return color image from greyscale image
{
    ofxCvColorImage output;
    output.setFromPixels(input.getPixels());
    int i = 0;
    while ( i < input.getPixels().size() ) {
        output.getPixels()[i] = operator()(output.getPixels()[i]);
        i++;
    }
	input.getPixels();
    /* Check for out-of-bounds arguments: */
	if(scalar<=min)
		return entries[0];
	else if(scalar>=max)
		return entries[numEntries-1];
	
	/* Calculate the base map index: */
	scalar=scalar*factor-offset;
	int index=int(floor(scalar));
	if(index==numEntries-1)
		--index;
	scalar-=double(index);
	Color result;
    result=entries[index]*(1.0-scalar)+entries[index+1]*scalar;
	
	return result;
}

