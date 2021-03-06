/***********************************************************************
 FrameFilter - Class to filter streams of depth frames arriving from a
 depth camera, with code to detect unstable values in each pixel, and
 fill holes resulting from invalid samples.
 Forked from Oliver Kreylos's Augmented Reality Sandbox (SARndbox).
 ***********************************************************************/

#include "FrameFilter.h"
#include "ofConstants.h"

/****************************
 Methods of class FrameFilter:
 ****************************/

FrameFilter::FrameFilter(): newFrame(true), bufferInitiated(false)
{
}

bool FrameFilter::setup(const unsigned int swidth,const unsigned int sheight,int sNumAveragingSlots, unsigned int newMinNumSamples, unsigned int newMaxVariance, float newHysteresis, bool newSpatialFilter, int sgradFieldresolution, float snearclip, float sfarclip, void* _backend)
{
	/* Settings variables : */
	width = swidth;
    height = sheight;
    gradFieldresolution = sgradFieldresolution;
	
	/* Initialize the valid depth range: */
	setValidDepthInterval(1,254);
	
	/* Initialize the averaging buffer: */
	numAveragingSlots=sNumAveragingSlots;
    
	/* Initialize the stability criterion: */
    //	minNumSamples=(numAveragingSlots+1)/2;
    //	maxVariance=4;
    //	hysteresis=0.1f;
	retainValids=true;
	instableValue=0.0;
    maxgradfield = 1000;
    
    nearclip = snearclip;
    farclip = sfarclip;
    depthrange = sfarclip-snearclip;
    
    minNumSamples=newMinNumSamples;
    maxVariance=newMaxVariance;
    hysteresis=newHysteresis;
	
	/* Enable spatial filtering: */
    //	spatialFilter=true;
    spatialFilter=newSpatialFilter;
    
	/* Initialize the gradient field vector*/
    gradFieldresolution = sgradFieldresolution;
    std::cout<< "Gradient Field resolution" << gradFieldresolution <<std::endl;
    gradFieldcols = width / sgradFieldresolution;
    std::cout<< "Width: " << width << " Cols: " << gradFieldcols <<std::endl;
    gradFieldrows = height / sgradFieldresolution;
    std::cout<< "Height: " << height << " Rows: " << gradFieldrows <<std::endl;
    
    // cast the kinect backend
    backend = static_cast <ofxKinect *>(_backend);
    // check if the backend is connected & capturing calibrated video
    if(!backend->isConnected()){
        ofLog(OF_LOG_ERROR, "Please open the kinect prior to setting the Framefilter");
        return;
    }
    
    //setting buffers
	initiateBuffers();
    
	return true;
}

FrameFilter::~FrameFilter(){
	// when the class is destroyed
	// close both channels and wait for
	// the thread to finish
    //	toAnalyze.close();
    //	analyzed.close();
    delete[] averagingBuffer;
    delete[] statBuffer;
    delete[] validBuffer;
    delete[] wrldcoordbuffer;
    delete[] gradField;
    //	waitForThread(true);
}

void FrameFilter::resetBuffers(void){
	/* Release all allocated buffers if needed */
    if (bufferInitiated){
        delete[] averagingBuffer;
        delete[] statBuffer;
        delete[] validBuffer;
        delete[] wrldcoordbuffer;
        delete[] gradField;
    }
    initiateBuffers();
}


void FrameFilter::initiateBuffers(void){
    //    /* Initialize the input frame slot: */
    //    inputFrameVersion=0;
    
    averagingBuffer=new RawDepth[numAveragingSlots*height*width];
    RawDepth* abPtr=averagingBuffer;
    for(int i=0;i<numAveragingSlots;++i)
        for(unsigned int y=0;y<height;++y)
            for(unsigned int x=0;x<width;++x,++abPtr)
                *abPtr=255; // Mark sample as invalid
    averagingSlotIndex=0;
    
    /* Initialize the statistics buffer: */
    statBuffer=new unsigned int[height*width*3];
    unsigned int* sbPtr=statBuffer;
    for(unsigned int y=0;y<height;++y)
        for(unsigned int x=0;x<width;++x)
            for(int i=0;i<3;++i,++sbPtr)
                *sbPtr=0;
    
    /* Initialize the valid buffer: */
    validBuffer=new RawDepth[height*width];
    RawDepth* vbPtr=validBuffer;
    for(unsigned int y=0;y<height;++y)
        for(unsigned int x=0;x<width;++x,++vbPtr)
            *vbPtr=0;
    
    /* Initialize the gradient field buffer: */
    gradField = new ofVec2f[gradFieldcols*gradFieldrows];
    ofVec2f* gfPtr=gradField;
    for(unsigned int y=0;y<gradFieldrows;++y)
        for(unsigned int x=0;x<gradFieldcols;++x,++gfPtr)
            *gfPtr=ofVec2f(0);
    
    /* Initialize the gradient field buffer: */
    wrldcoordbuffer = new Point3f[height*width];
    Point3f* wcPtr=wrldcoordbuffer;
    for(unsigned int y=0;y<height;++y)
        for(unsigned int x=0;x<width;++x,++wcPtr)
            *wcPtr=Point3f(0, 0, 0);
    
    bufferInitiated = true;
}
void FrameFilter::setDepthRange(float snearclip, float sfarclip){
    // send the frame to the thread for analyzing
    // this makes a copy but we can't avoid it anyway if
    // we want to update the grabber while analyzing
    // previous frames
    //    ++inputFrameVersion;
    //    toAnalyze.send(inputframe);
    nearclip = snearclip;
    farclip = sfarclip;
    depthrange = sfarclip-snearclip;
}

void FrameFilter::update(){
    // check if there's a new analyzed frame and upload
    // it to the texture. we use a while loop to drop any
    // extra frame in case the main thread is slower than
    // the analysis
    // tryReceive doesn't reallocate or make any copies
    //	newFrame = false;
    //	while(analyzed.tryReceive(inputframe)){
    //		newFrame = true;
    //	}
    //	if(newFrame){
    //        if(!texture.isAllocated()){
    //            texture.allocate(inputframe);
    //        }
    //		texture.loadData(inputframe);
    //	}
}

bool FrameFilter::isFrameNew(){
    return newFrame;
}

ofVec2f FrameFilter::getGradFieldXY(int x, int y){
    int xbin = (int)floorf(((float)x)/(float)gradFieldresolution);
    int ybin = (int)floorf(((float)y)/(float)gradFieldresolution);
    return gradField[ybin*gradFieldcols+xbin];
}

ofVec2f* FrameFilter::getGradField(){
    return gradField;
}

Point3f* FrameFilter::getWrldcoordbuffer(){
    return wrldcoordbuffer;
}

void FrameFilter::displayFlowField()
{
    
    /*
     Uncomment to draw an x at the centre of the screen
     ofPoint screenCenter = ofPoint(ofGetWidth() / 2,ofGetHeight() / 2);
     ofSetColor(255,0,0,255);
     ofLine(screenCenter.x - 50,screenCenter.y-50,screenCenter.x+50,screenCenter.y+50);
     ofLine(screenCenter.x + 50,screenCenter.y-50,screenCenter.x-50,screenCenter.y+50);
     */
    
    for(int rowPos=0; rowPos< gradFieldrows ; rowPos++)
    {
        for(int colPos=0; colPos< gradFieldcols ; colPos++)
        {
            ofFill();
            ofPushMatrix();
            // add half resolution to each dimension to put us in center of each 'cell'
            ofTranslate((colPos*gradFieldresolution) + gradFieldresolution/2,rowPos*gradFieldresolution  + gradFieldresolution/2);
            ofVec2f v1(1,0);
            ofVec2f v2 = gradField[colPos + (rowPos * gradFieldcols)];
            //float angleToRotate = v1.angle(v2);
            //ofRotate(angleToRotate);
            drawArrow(v2*0.1);//(colPos + (rowPos * gradFieldcols))/10);//
            ofSetColor(0,0,255,255);
            //int angFloored = angleToRotate;
            // ofDrawBitmapString(ofToString(angFloored), 0,0); // uncomment to output angle at position for debugging
            ofPopMatrix();
        }
    }
}

void FrameFilter::drawArrow(ofVec2f v1)
{
    // half the length is subtracted from the x point positions to move the rotation axis to the center
    ofSetColor(255,0,0,255);
    ofDrawLine(0, 0, v1.x, v1.y);
    ofDrawCircle(v1.x, v1.y, 5);//(-length/2 + length*0.8, length*0.1, length/2, 0);
    //ofDrawLine(-length/2 + length*0.8, length*-0.1, length/2, 0);
}

ofPixels FrameFilter::filter(ofPixels inputframe){
    // wait until there's a new frame
    // this blocks the thread, so it doesn't use
    // the CPU at all, until a frame arrives.
    // also receive doesn't allocate or make any copies
    //    ofPixels inputframe;
    //    inputframe.setImageType(OF_IMAGE_GRAYSCALE);
    //    while(toAnalyze.receive(inputframe)){
    //        // we have a new frame, process it, the analysis
    //        // here is just a thresholding for the sake of
    //        // simplicity
    //        unsigned int lastInputFrameVersion=0;
    //        lastInputFrameVersion=inputFrameVersion;
    //
    //   return inputframe; ///////////////////////////////////////////////////////////////////
    
    // Convert in proj space
    //    ofPixels inputframe = convertProjSpace(sinputframe);
    
    // Create a new output frame: */
    ofPixels newOutputFrame;
    newOutputFrame.allocate(width, height, 1);
    
    /* Initialize a new gradient field buffer and number of valid gradient measures */
    //    ofVec2f* valgradField = new ofVec2f[gradFieldcols*gradFieldrows];
    //    ofVec2f* newgradField = new ofVec2f[gradFieldcols*gradFieldrows];
    //        ofVec2f* vlfPtr=valgradField;
    //        ofVec2f* ngfPtr=newgradField;
    //        for(unsigned int y=0;y<gradFieldrows;++y)
    //            for(unsigned int x=0;x<gradFieldcols;++x,++ngfPtr,++vlfPtr) {
    //                *ngfPtr=ofVec2f(0);
    //                *vlfPtr=ofVec2f(0);
    //            }
    
    
    // Enter the new frame into the averaging buffer and calculate the output frame's pixel values: */
    const RawDepth* ifPtr=static_cast<const RawDepth*>(inputframe.getData());
    RawDepth* abPtr=averagingBuffer+averagingSlotIndex*height*width;
    unsigned int* sPtr=statBuffer;
    RawDepth* ofPtr=validBuffer; // static_cast<const float*>(outputFrame.getBuffer());
    RawDepth* nofPtr=static_cast<RawDepth*>(newOutputFrame.getData());
    float z;
    
    for(unsigned int y=0;y<height;++y)
    {
        //            float py=float(y)+0.5f;
        for(unsigned int x=0;x<width;++x,++ifPtr,++abPtr,sPtr+=3,++ofPtr,++nofPtr)
        {
            //                float px=float(x)+0.5f;
            
            unsigned char oldVal=*abPtr;
            unsigned char newVal=*ifPtr;
            
            //                    /* Depth-correct the new value: */
            //                    float newCVal=pdcPtr->correct(newVal);
            //
            //                    /* Plug the depth-corrected new value into the minimum and maximum plane equations to determine its validity: */
            //                    float minD=minPlane[0]*px+minPlane[1]*py+minPlane[2]*newCVal+minPlane[3];
            //                    float maxD=maxPlane[0]*px+maxPlane[1]*py+maxPlane[2]*newCVal+maxPlane[3];
            if(newVal != 0 && newVal != 255) // Pixel depth not clipped => inside valide range
            {
                /* Store the new input value: */
                *abPtr=newVal;
                
                /* Update the pixel's statistics: */
                ++sPtr[0]; // Number of valid samples
                sPtr[1]+=newVal; // Sum of valid samples
                sPtr[2]+=newVal*newVal; // Sum of squares of valid samples
                
                /* Check if the previous value in the averaging buffer was valid: */
                if(oldVal!=255)
                {
                    --sPtr[0]; // Number of valid samples
                    sPtr[1]-=oldVal; // Sum of valid samples
                    sPtr[2]-=oldVal*oldVal; // Sum of squares of valid samples
                }
            }
            else if(!retainValids)
            {
                /* Store an invalid input value: */
                *abPtr=255;
                
                /* Check if the previous value in the averaging buffer was valid: */
                if(oldVal!=255)
                {
                    --sPtr[0]; // Number of valid samples
                    sPtr[1]-=oldVal; // Sum of valid samples
                    sPtr[2]-=oldVal*oldVal; // Sum of squares of valid samples
                }
            }
            
            // Check if the pixel is considered "stable": */
            if(sPtr[0]>=minNumSamples&&sPtr[2]*sPtr[0]<=maxVariance*sPtr[0]*sPtr[0]+sPtr[1]*sPtr[1])
            {
                /* Check if the new depth-corrected running mean is outside the previous value's envelope: */
                float newFiltered=float(sPtr[1])/float(sPtr[0]);
                if(abs(newFiltered-*ofPtr)>=hysteresis)
                {
                    /* Set the output pixel value to the depth-corrected running mean: */
                    *nofPtr=*ofPtr=newFiltered;
                    // Update world coordonate of point
//                    z = (255.0-newFiltered)/255.0*(farclip-nearclip)+nearclip;
//                    wrldcoordbuffer[y*width+x]=toCv(backend->getWorldCoordinateAt(x, y, z));
                }
                else
                {
                    /* Leave the pixel at its previous value: */
                    *nofPtr=*ofPtr;
                }
            }
            else if(retainValids)
            {
                /* Leave the pixel at its previous value: */
                *nofPtr=*ofPtr;
            }
            else
            {
                /* Assign default value to instable pixels: */
                *nofPtr=instableValue;
            }
        }
    }
    
    /* Go to the next averaging slot: */
    if(++averagingSlotIndex==numAveragingSlots)
        averagingSlotIndex=0;
    
    /* Apply a spatial filter if requested: */
    if(spatialFilter)
    {
        for(int filterPass=0;filterPass<2;++filterPass)
        {
            /* Low-pass filter the entire output frame in-place: */
            for(unsigned int x=0;x<width;++x)
            {
                /* Get a pointer to the current column: */
                RawDepth* colPtr=static_cast<RawDepth*>(newOutputFrame.getData())+x;
                
                /* Filter the first pixel in the column: */
                float lastVal=*colPtr;
                *colPtr=(colPtr[0]*2.0f+colPtr[width])/3.0f;
                colPtr+=width;
                
                /* Filter the interior pixels in the column: */
                for(unsigned int y=1;y<height-1;++y,colPtr+=width)
                {
                    /* Filter the pixel: */
                    float nextLastVal=*colPtr;
                    *colPtr=(lastVal+colPtr[0]*2.0f+colPtr[width])*0.25f;
                    lastVal=nextLastVal;
                }
                
                /* Filter the last pixel in the column: */
                *colPtr=(lastVal+colPtr[0]*2.0f)/3.0f;
            }
            RawDepth* rowPtr=static_cast<RawDepth*>(newOutputFrame.getData());
            for(unsigned int y=0;y<height;++y)
            {
                /* Filter the first pixel in the row: */
                float lastVal=*rowPtr;
                *rowPtr=(rowPtr[0]*2.0f+rowPtr[1])/3.0f;
                ++rowPtr;
                
                /* Filter the interior pixels in the row: */
                for(unsigned int x=1;x<width-1;++x,++rowPtr)
                {
                    /* Filter the pixel: */
                    float nextLastVal=*rowPtr;
                    *rowPtr=(lastVal+rowPtr[0]*2.0f+rowPtr[1])*0.25f;
                    lastVal=nextLastVal;
                }
                
                /* Filter the last pixel in the row: */
                *rowPtr=(lastVal+rowPtr[0]*2.0f)/3.0f;
                ++rowPtr;
            }
        }
    }
    
    /* Pass the new output frame to the registered receiver: */
    //            if(outputFrameFunction!=0)
    //                (*outputFrameFunction)(newOutputFrame);
    
    /* Retain the new output frame: */
    //        float maxx = 0;
    //        float minn = 1000;
    //        for(unsigned int y=0;y<height*height;++y)
    //        {
    //            if (newOutputFrame.getData()[y]>maxx)
    //                maxx =newOutputFrame.getData()[y];
    //            if (newOutputFrame.getData()[y]<minn && newOutputFrame.getData()[y]!=0)
    //                minn =newOutputFrame.getData()[y];
    //        }
    
    outputframe=newOutputFrame;
    updateGradientField();
    // once processed send the result back to the
    // main thread. in c++11 we can move it to
    // avoid a copy
    return newOutputFrame;
    //#if __cplusplus>=201103
    //        analyzed.send(std::move(newOutputFrame));
    //#else
    //        analyzed.send(newOutputFrame);
    //#endif
    //    }
}

void FrameFilter::updateGradientField()
{
    //Compute gradient field
    int ind = 0;
    float gx;
    float gy;
    int gvx, gvy;
    float lgth = 0;
    RawDepth* nofPtr=outputframe.getData();
    for(unsigned int y=0;y<gradFieldrows;++y) {
        for(unsigned int x=0;x<gradFieldcols;++x) {
            //            if (x==7 && y ==7)
            //                cout << "hop" << endl;
            gx = 0;
            gvx = 0;
            gy = 0;
            gvy = 0;
            for (unsigned int i=0; i<gradFieldresolution; i++) {
                ind = y*gradFieldresolution*width+i*width+x*gradFieldresolution;
                if (nofPtr[ind]!= 0 && nofPtr[ind+gradFieldresolution-1]!=0){
                    gvx+=1;
                    gx+=nofPtr[ind]-nofPtr[ind+gradFieldresolution-1];
                }
                ind = y*gradFieldresolution*width+i+x*gradFieldresolution;
                if (nofPtr[ind]!= 0 && nofPtr[ind+(gradFieldresolution-1)*width]!=0){
                    gvy+=1;
                    gy+=nofPtr[ind]-nofPtr[ind+(gradFieldresolution-1)*width];
                }
            }
            if (gvx !=0 && gvy !=0)
                gradField[y*gradFieldcols+x]=ofVec2f(gx/gradFieldresolution/gvx*depthrange, gy/gradFieldresolution/gvy*depthrange);
            if (gradField[y*gradFieldcols+x].length() > maxgradfield){
                gradField[y*gradFieldcols+x].scale(maxgradfield);// /= gradField[y*gradFieldcols+x].length()*maxgradfield;
                lgth+=1;
            }
        }
    }
    //    cout << "Max gradient: " << lgth << endl;
}


void FrameFilter::setValidDepthInterval(unsigned int newMinDepth,unsigned int newMaxDepth)
{
    /* Set the equations for the minimum and maximum plane in depth image space: */
    //	minPlane[0]=0.0f;
    //	minPlane[1]=0.0f;
    //	minPlane[2]=1.0f;
    //	minPlane[3]=-float(newMinDepth)+0.5f;
    //	maxPlane[0]=0.0f;
    //	maxPlane[1]=0.0f;
    //	maxPlane[2]=1.0f;
    //	maxPlane[3]=-float(newMaxDepth)-0.5f;
    min=newMinDepth;
    max=newMaxDepth;
}

void FrameFilter::setStableParameters(unsigned int newMinNumSamples,unsigned int newMaxVariance)
{
    minNumSamples=newMinNumSamples;
    maxVariance=newMaxVariance;
}

void FrameFilter::setHysteresis(float newHysteresis)
{
    hysteresis=newHysteresis;
}

void FrameFilter::setRetainValids(bool newRetainValids)
{
    retainValids=newRetainValids;
}

void FrameFilter::setInstableValue(float newInstableValue)
{
    instableValue=newInstableValue;
}

void FrameFilter::setSpatialFilter(bool newSpatialFilter)
{
    spatialFilter=newSpatialFilter;
}

//void FrameFilter::setOutputFrameFunction(FrameFilter::OutputFrameFunction* newOutputFrameFunction)
//	{
//	delete outputFrameFunction;
//	outputFrameFunction=newOutputFrameFunction;
//	}

//void FrameFilter::receiveRawFrame(const Kinect::FrameBuffer& newFrame)
//	{
//	Threads::MutexCond::Lock inputLock(inputCond);
//
//	/* Store the new buffer in the input buffer: */
//	inputFrame=newFrame;
//	++inputFrameVersion;
//
//	/* Signal the background thread: */
//	inputCond.signal();
//	}
