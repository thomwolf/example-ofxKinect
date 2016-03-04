/***********************************************************************
SurfaceRenderer - Class to render a surface defined by a regular grid in
depth image space.
Adapted from the Augmented Reality Sandbox (SARndbox) by Oliver Kreylos
***********************************************************************/

#include "SurfaceRenderer.h"

#include <string>
#include <vector>
#include <iostream>

SurfaceRenderer::SurfaceRenderer(const unsigned int swidth,const unsigned int sheight,const SurfaceRenderer::PTransform& sDepthProjection,const SurfaceRenderer::Plane& sBasePlane)
	:depthProjection(sDepthProjection),
	 basePlane(sBasePlane),
	 usePreboundDepthTexture(false),
	 drawContourLines(true),contourLineFactor(1.0f),
	 useHeightMap(true),heightMapScale(1.0f),heightMapOffset(0.0f),
	 surfaceSettingsVersion(1),
	 waterOpacity(2.0f),
	 depthImageVersion(1),
	 animationTime(0.0)
	{
	
	/* Copy the depth image size: */
        width = swidth;
        height = sheight;
        
	/* Convert the base plane to a homogeneous plane equation: */
	for(int i=0;i<3;++i)
		basePlaneEq[i]=GLfloat(basePlane.getNormal()[i]);
	basePlaneEq[3]=GLfloat(-basePlane.getOffset());
	
	/* Initialize the depth image: */
       depthImage.allocate(width, height, 1);
        depthImage.set(0);
    }

void SurfaceRenderer::initContext()
	{
	
    // Initialise mesh
        mesh.clear();
 	for(unsigned int y=0;y<height;++y)
		for(unsigned int x=0;x<width;++x)
			{
                mesh.addVertex(ofPoint(float(x)+0.5f,float(y)+0.5f,0.0f)); // make a new vertex
			}
    for(unsigned int y=1;y<height;++y)
		for(unsigned int x=0;x<width;++x)
			{
                mesh.addIndex(y*width+x);
                mesh.addIndex((y-1)*width+x);
			}
	
	// Initialize the depth image texture:
        depthTexture.allocate(depthImage);
	
	// Load shaders
    elevationShader.load( "SurfaceElevationShader.vs", "SurfaceElevationShader.fs" );
    heightMapShader.load("heightMapShader.vs", "heightMapShader.fs");
        

	}

void SurfaceRenderer::setUsePreboundDepthTexture(bool newUsePreboundDepthTexture)
	{
	usePreboundDepthTexture=newUsePreboundDepthTexture;
	}

void SurfaceRenderer::setDrawContourLines(bool newDrawContourLines)
	{
	drawContourLines=newDrawContourLines;
	++surfaceSettingsVersion;
	}

void SurfaceRenderer::setContourLineDistance(GLfloat newContourLineDistance)
	{
	/* Set the new contour line factor: */
	contourLineFactor=1.0f/newContourLineDistance;
	}

void SurfaceRenderer::setUseHeightMap(bool newUseHeightMap)
	{
	useHeightMap=newUseHeightMap;
	++surfaceSettingsVersion;
	}

void SurfaceRenderer::setHeightMapRange(GLsizei newHeightMapSize,GLfloat newMinElevation,GLfloat newMaxElevation)
	{
	/* Calculate the new height map elevation scaling and offset coefficients: */
	GLdouble hms=GLdouble(newHeightMapSize-1)/((newMaxElevation-newMinElevation)*GLdouble(newHeightMapSize));
	GLdouble hmo=0.5/GLdouble(newHeightMapSize)-hms*newMinElevation;
	
	heightMapScale=GLfloat(hms);
	heightMapOffset=GLfloat(hmo);
	}

void SurfaceRenderer::setDepthImage(const ofFloatPixels newDepthImage)
	{
	/* Update the depth image: */
	depthImage=newDepthImage;
	++depthImageVersion;
	}

void SurfaceRenderer::setAnimationTime(double newAnimationTime)
	{
	/* Set the new animation time: */
	animationTime=newAnimationTime;
	
	}

void SurfaceRenderer::glPrepareContourLines()
	{
	/*********************************************************************
	Prepare the half-pixel-offset frame buffer for subsequent per-fragment
	Marching Squares contour line extraction.
	*********************************************************************/
	
	/* Query the current viewport: */
	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT,viewport);
	
	/* Save the currently-bound frame buffer and clear color: */
	GLint currentFrameBuffer;
	glGetIntegerv(GL_FRAMEBUFFER_BINDING_EXT,&currentFrameBuffer);
	GLfloat currentClearColor[4];
	glGetFloatv(GL_COLOR_CLEAR_VALUE,currentClearColor);
	
	/* Check if the contour line frame buffer needs to be created or resized */
        if (!contourLineFramebufferObject.isAllocated()||contourLineFramebufferSize[0]!=viewport[2]+1||contourLineFramebufferSize[1]!=viewport[3]+1) {
            for(int i=0;i<2;++i)
                contourLineFramebufferSize[i]=viewport[2+i]+1;
            contourLineFramebufferObject.allocate(contourLineFramebufferSize[0], contourLineFramebufferSize[1]);
        }
	
	/* Extend the viewport to render the corners of the final pixels: */
	glViewport(0,0,viewport[2]+1,viewport[3]+1);
	glClearColor(0.0f,0.0f,0.0f,1.0f);
	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
	
	/* Adjust the projection matrix to render the corners of the final pixels: */
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	GLdouble proj[16];
	glGetDoublev(GL_PROJECTION_MATRIX,proj);
	GLdouble xs=GLdouble(viewport[2])/GLdouble(viewport[2]+1);
	GLdouble ys=GLdouble(viewport[3])/GLdouble(viewport[3]+1);
	for(int j=0;j<4;++j)
		{
		proj[j*4+0]*=xs;
		proj[j*4+1]*=ys;
		}
	glLoadIdentity();
	glMultMatrixd(proj);
	
	/*********************************************************************
	Render the surface's elevation into the half-pixel offset frame
	buffer.
	*********************************************************************/
	
	/* start the elevation shader and contourLineFramebufferObject: */
    contourLineFramebufferObject.begin();
	elevationShader.begin();
	
	/* Set up the depth image texture: */
	if(!usePreboundDepthTexture)
		{
		
		/* Check if the texture is outdated: */
		if(depthTextureVersion!=depthImageVersion)
			{
			/* Upload the new depth texture: */
                depthTexture.loadData(depthImage);
			//glTexSubImage2D(GL_TEXTURE_RECTANGLE_ARB,0,0,0,width,height,GL_LUMINANCE,GL_FLOAT,depthImage.getBuffer());
			
			/* Mark the depth texture as current: */
			depthTextureVersion=depthImageVersion;
			}
		}
	
        elevationShader.setUniformTexture( "depthSampler", depthTexture, 1 ); //"1" means that it is texture 1
	/* Upload the base plane equation: */
        elevationShader.setUniform4f("basePlane",basePlaneEq);
	
	/* Draw the surface: */
        mesh.draw();
	
    contourLineFramebufferObject.end();
    elevationShader.end();
	
	/*********************************************************************
	Restore previous OpenGL state.
	*********************************************************************/
	
	/* Restore the original viewport and projection matrix: */
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glViewport(viewport[0],viewport[1],viewport[2],viewport[3]);
	
	/* Restore the original clear color and frame buffer binding: */
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT,currentFrameBuffer);
	glClearColor(currentClearColor[0],currentClearColor[1],currentClearColor[2],currentClearColor[3]);
	}

void SurfaceRenderer::glRenderSinglePass(ofTexture heightColorMapTexture)
	{

	/* Check if contour line rendering is enabled: */
	if(drawContourLines)
		{
		/* Run the first rendering pass to create a half-pixel offset texture of surface elevations: */
		glPrepareContourLines();
		}

	/* Bind the single-pass surface shader: */
        heightMapShader.begin();
	
	/* Set up the depth image texture: */
	if(!usePreboundDepthTexture)
		{
		
		/* Check if the texture is outdated: */
		if(depthTextureVersion!=depthImageVersion)
			{
			/* Upload the new depth texture: */
                depthTexture.loadData(depthImage);
			
			/* Mark the depth texture as current: */
			depthTextureVersion=depthImageVersion;
			}
		}
        heightMapShader.setUniformTexture( "depthSampler", depthTexture, 1 ); //"1" means that it is texture 1
        heightMapShader.setUniform4f("basePlane",basePlaneEq);
	
	/* Upload the depth projection matrix: */
	
	if(useHeightMap)
		{
		/* Upload the base plane equation: */
		
		/* Upload the height color map texture coordinate transformation: */
		heightMapShader.setUniform2f("heightColorMapTransformation",ofVec2f(heightMapScale,heightMapOffset));
		
		/* Bind the height color map texture: */
		heightMapShader.setUniformTexture("heightColorMapSampler",heightColorMapTexture, 2);
		}
	
	if(drawContourLines)
		{
		/* Bind the pixel corner elevation texture: */
		heightMapShader.setUniformTexture("pixelCornerElevationSampler", contourLineFramebufferObject.getTexture(), 3);
		
		/* Upload the contour line distance factor: */
		heightMapShader.setUniform1f("contourLineFactor",contourLineFactor);
		}

	/* Draw the surface: */
        mesh.draw();
        heightMapShader.end();
	}

