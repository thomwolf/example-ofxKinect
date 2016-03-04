/***********************************************************************
Sandbox - Vrui application to drive an augmented reality sandbox.
Copyright (c) 2012-2015 Oliver Kreylos

This file is part of the Augmented Reality Sandbox (SARndbox).

The Augmented Reality Sandbox is free software; you can redistribute it
and/or modify it under the terms of the GNU General Public License as
published by the Free Software Foundation; either version 2 of the
License, or (at your option) any later version.

The Augmented Reality Sandbox is distributed in the hope that it will be
useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public License along
with the Augmented Reality Sandbox; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
***********************************************************************/

#include "Sandbox.h"

#include <string.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <stdexcept>
#include <iostream>
#include <Misc/SelfDestructPointer.h>
#include <Misc/FunctionCalls.h>
#include <Misc/FileNameExtensions.h>
#include <IO/File.h>
#include <IO/ValueSource.h>
#include <Math/Math.h>
#include <Math/Constants.h>
#include <Geometry/Point.h>
#include <Geometry/AffineCombiner.h>
#include <Geometry/HVector.h>
#include <Geometry/Plane.h>
#include <Geometry/GeometryValueCoders.h>
#include <Geometry/OutputOperators.h>
#include <GL/gl.h>
#include <GL/GLMaterialTemplates.h>
#include <GL/GLPrintError.h>
#include <GL/GLColorMap.h>
#include <GL/GLLightTracker.h>
#include <GL/Extensions/GLEXTFramebufferObject.h>
#include <GL/Extensions/GLARBTextureRectangle.h>
#include <GL/Extensions/GLARBTextureFloat.h>
#include <GL/Extensions/GLARBTextureRg.h>
#include <GL/Extensions/GLARBDepthTexture.h>
#include <GL/Extensions/GLARBShaderObjects.h>
#include <GL/Extensions/GLARBVertexShader.h>
#include <GL/Extensions/GLARBFragmentShader.h>
#include <GL/Extensions/GLARBMultitexture.h>
#include <GL/GLContextData.h>
#include <GL/GLGeometryWrappers.h>
#include <GL/GLTransformationWrappers.h>
#include <GLMotif/PopupMenu.h>
#include <GLMotif/Menu.h>
#include <Vrui/Vrui.h>
#include <Vrui/Lightsource.h>
#include <Vrui/LightsourceManager.h>
#include <Vrui/Viewer.h>
#include <Vrui/ToolManager.h>
#include <Vrui/DisplayState.h>
#include <Vrui/OpenFile.h>
#include <Kinect/Camera.h>

#define SAVEDEPTH 0

#if SAVEDEPTH
#include <Images/RGBImage.h>
#include <Images/WriteImageFile.h>
#endif

#include "FrameFilter.h"
#include "SurfaceRenderer.h"
#include "WaterTable2.h"

/************************
Methods of class Sandbox:
************************/

void Sandbox::rawDepthFrameDispatcher(const Kinect::FrameBuffer& frameBuffer)
	{
	/* Pass the received frame to the frame filter and the rain maker's frame filter: */
	if(frameFilter!=0&&!pauseUpdates)
		frameFilter->receiveRawFrame(frameBuffer);
	if(rmFrameFilter!=0)
		rmFrameFilter->receiveRawFrame(frameBuffer);
	}

void Sandbox::receiveFilteredFrame(const Kinect::FrameBuffer& frameBuffer)
	{
	/* Put the new frame into the frame input buffer: */
	filteredFrames.postNewValue(frameBuffer);
	
	/* Wake up the foreground thread: */
	Vrui::requestUpdate();
	}

void Sandbox::pauseUpdatesCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData)
	{
	pauseUpdates=cbData->set;
	}

GLMotif::PopupMenu* Sandbox::createMainMenu(void)
	{
	/* Create a popup shell to hold the main menu: */
	GLMotif::PopupMenu* mainMenuPopup=new GLMotif::PopupMenu("MainMenuPopup",Vrui::getWidgetManager());
	mainMenuPopup->setTitle("AR Sandbox");
	
	/* Create the main menu itself: */
	GLMotif::Menu* mainMenu=new GLMotif::Menu("MainMenu",mainMenuPopup,false);
	
	/* Create a button to pause topography updates: */
	GLMotif::ToggleButton* pauseUpdatesButton=new GLMotif::ToggleButton("PauseUpdatesButton",mainMenu,"Pause Topography");
	pauseUpdatesButton->setToggle(false);
	pauseUpdatesButton->getValueChangedCallbacks().add(this,&Sandbox::pauseUpdatesCallback);
	
	/* Finish building the main menu: */
	mainMenu->manageChild();
	
	return mainMenuPopup;
	}

Sandbox::Sandbox(int& argc,char**& argv,char**& appDefaults)
	:Vrui::Application(argc,argv,appDefaults),
	 camera(0),
	 frameFilter(0),pauseUpdates(false),
	 surfaceMaterial(GLMaterial::Color(0.8f,0.8f,0.8f),GLMaterial::Color(1.0f,1.0f,1.0f),25.0f),
	 surfaceRenderer(0),
	 waterTable(0),waterSpeed(1.0),waterMaxSteps(30),rainStrength(0.25f),
	 rmFrameFilter(0),rainMaker(0),addWaterFunction(0),addWaterFunctionRegistered(false),
	 fixProjectorView(false),hillshade(false),useShadows(false),useHeightMap(false),
	 waterRenderer(0),
	 sun(0),
	 mainMenu(0)
	{
	/* Initialize the custom tool classes: */
	WaterTool::initClass(*Vrui::getToolManager());
	LocalWaterTool::initClass(*Vrui::getToolManager());
	
	/* Process command line parameters: */
	bool printHelp=false;
	int cameraIndex=0;
	std::string sandboxLayoutFileName=CONFIGDIR;
	sandboxLayoutFileName.push_back('/');
	sandboxLayoutFileName.append("BoxLayout.txt");
	bool useHeightMap=true;
	std::string heightColorMapFileName=CONFIGDIR;
	heightColorMapFileName.push_back('/');
	heightColorMapFileName.append(DEFAULTHEIGHTCOLORMAPNAME);
	double elevationMin=-1000.0;
	double elevationMax=1000.0;
	unsigned int validMin=0;
	unsigned int validMax=2047;
	int numAveragingSlots=30;
	unsigned int minNumSamples=10;
	unsigned int maxVariance=2;
	float hysteresis=0.1f;
	bool useContourLines=true;
	GLfloat contourLineSpacing=0.75f;
	unsigned int wtSize[2];
	wtSize[0]=640U;
	wtSize[1]=480U;
	GLfloat waterOpacity=2.0f;
	bool renderWaterSurface=false;
	double rainElevationMin=-1000.0;
	double rainElevationMax=1000.0;
	double evaporationRate=0.0;
	for(int i=1;i<argc;++i)
		{
		if(argv[i][0]=='-')
			{
			if(strcasecmp(argv[i]+1,"h")==0)
				printHelp=true;
			else if(strcasecmp(argv[i]+1,"c")==0)
				{
				++i;
				cameraIndex=atoi(argv[i]);
				}
			else if(strcasecmp(argv[i]+1,"slf")==0)
				{
				++i;
				sandboxLayoutFileName=argv[i];
				}
			else if(strcasecmp(argv[i]+1,"er")==0)
				{
				++i;
				elevationMin=atof(argv[i]);
				++i;
				elevationMax=atof(argv[i]);
				}
			else if(strcasecmp(argv[i]+1,"nas")==0)
				{
				++i;
				numAveragingSlots=atoi(argv[i]);
				}
			else if(strcasecmp(argv[i]+1,"sp")==0)
				{
				++i;
				minNumSamples=atoi(argv[i]);
				++i;
				maxVariance=atoi(argv[i]);
				}
			else if(strcasecmp(argv[i]+1,"he")==0)
				{
				++i;
				hysteresis=float(atof(argv[i]));
				}
			else if(strcasecmp(argv[i]+1,"nhm")==0)
				{
				useHeightMap=false;
				}
			else if(strcasecmp(argv[i]+1,"hcm")==0)
				{
				++i;
				heightColorMapFileName=argv[i];
				}
			else if(strcasecmp(argv[i]+1,"ncl")==0)
				{
				useContourLines=false;
				}
			else if(strcasecmp(argv[i]+1,"cls")==0)
				{
				++i;
				contourLineSpacing=GLfloat(atof(argv[i]));
				}
			else if(strcasecmp(argv[i]+1,"wts")==0)
				{
				for(int j=0;j<2;++j)
					{
					++i;
					wtSize[j]=(unsigned int)(atoi(argv[i]));
					}
				}
			else if(strcasecmp(argv[i]+1,"ws")==0)
				{
				++i;
				waterSpeed=atof(argv[i]);
				++i;
				waterMaxSteps=atoi(argv[i]);
				}
			else if(strcasecmp(argv[i]+1,"wo")==0)
				{
				++i;
				waterOpacity=GLfloat(atof(argv[i]));
				}
			else if(strcasecmp(argv[i]+1,"rer")==0)
				{
				++i;
				rainElevationMin=atof(argv[i]);
				++i;
				rainElevationMax=atof(argv[i]);
				}
			else if(strcasecmp(argv[i]+1,"rs")==0)
				{
				++i;
				rainStrength=GLfloat(atof(argv[i]));
				}
			else if(strcasecmp(argv[i]+1,"evr")==0)
				{
				++i;
				evaporationRate=atof(argv[i]);
				}
			else if(strcasecmp(argv[i]+1,"fpv")==0)
				fixProjectorView=true;
			else if(strcasecmp(argv[i]+1,"hs")==0)
				hillshade=true;
			else if(strcasecmp(argv[i]+1,"us")==0)
				useShadows=true;
			else if(strcasecmp(argv[i]+1,"uhm")==0)
				useHeightMap=true;
			else if(strcasecmp(argv[i]+1,"rws")==0)
				renderWaterSurface=true;
			}
		}
	
	if(printHelp)
		{
		std::cout<<"Usage: SARndbox [option 1] ... [option n]"<<std::endl;
		std::cout<<"  Options:"<<std::endl;
		std::cout<<"  -h"<<std::endl;
		std::cout<<"     Prints this help message"<<std::endl;
		std::cout<<"  -c <camera index>"<<std::endl;
		std::cout<<"     Selects the local Kinect camera of the given index (0: first camera"<<std::endl;
		std::cout<<"     on USB bus)"<<std::endl;
		std::cout<<"     Default: 0"<<std::endl;
		std::cout<<"  -slf <sandbox layout file name>"<<std::endl;
		std::cout<<"     Loads the sandbox layout file of the given name"<<std::endl;
		std::cout<<"     Default: "<<CONFIGDIR<<"/BoxLayout.txt"<<std::endl;
		std::cout<<"  -er <min elevation> <max elevation>"<<std::endl;
		std::cout<<"     Sets the range of valid sand surface elevations relative to the"<<std::endl;
		std::cout<<"     ground plane in cm"<<std::endl;
		std::cout<<"     Default: Range of elevation color map"<<std::endl;
		std::cout<<"  -nas <num averaging slots>"<<std::endl;
		std::cout<<"     Sets the number of averaging slots in the frame filter; latency is"<<std::endl;
		std::cout<<"     <num averaging slots> * 1/30 s"<<std::endl;
		std::cout<<"     Default: 30"<<std::endl;
		std::cout<<"  -sp <min num samples> <max variance>"<<std::endl;
		std::cout<<"     Sets the frame filter parameters minimum number of valid samples"<<std::endl;
		std::cout<<"     and maximum sample variance before convergence"<<std::endl;
		std::cout<<"     Default: 10 2"<<std::endl;
		std::cout<<"  -he <hysteresis envelope>"<<std::endl;
		std::cout<<"     Sets the size of the hysteresis envelope used for jitter removal"<<std::endl;
		std::cout<<"     Default: 0.1"<<std::endl;
		std::cout<<"  -nhm"<<std::endl;
		std::cout<<"     Disables elevation color mapping"<<std::endl;
		std::cout<<"  -hcm <elevation color map file name>"<<std::endl;
		std::cout<<"     Sets the name of the elevation color map"<<std::endl;
		std::cout<<"     Default: "<<CONFIGDIR<<"/"<<DEFAULTHEIGHTCOLORMAPNAME<<std::endl;
		std::cout<<"  -ncl"<<std::endl;
		std::cout<<"     Disables topographic contour lines"<<std::endl;
		std::cout<<"  -cls <contour line spacing>"<<std::endl;
		std::cout<<"     Sets the elevation distance between adjacent topographic contour"<<std::endl;
		std::cout<<"     lines in cm"<<std::endl;
		std::cout<<"     Default: 0.75"<<std::endl;
		std::cout<<"  -wts <water grid width> <water grid height>"<<std::endl;
		std::cout<<"     Sets the width and height of the water flow simulation grid"<<std::endl;
		std::cout<<"     Default: 640 480"<<std::endl;
		std::cout<<"  -ws <water speed> <water max steps>"<<std::endl;
		std::cout<<"     Sets the relative speed of the water simulation and the maximum"<<std::endl;
		std::cout<<"     number of simulation steps per frame"<<std::endl;
		std::cout<<"     Default: 1.0 30"<<std::endl;
		std::cout<<"  -wo <water opacity>"<<std::endl;
		std::cout<<"     Sets the water depth at which water appears opaque in cm"<<std::endl;
		std::cout<<"     Default: 2.0"<<std::endl;
		std::cout<<"  -rer <min rain elevation> <max rain elevation>"<<std::endl;
		std::cout<<"     Sets the elevation range of the rain cloud level relative to the"<<std::endl;
		std::cout<<"     ground plane in cm"<<std::endl;
		std::cout<<"     Default: Above range of elevation color map"<<std::endl;
		std::cout<<"  -rs <rain strength>"<<std::endl;
		std::cout<<"     Sets the strength of global or local rainfall in cm/s"<<std::endl;
		std::cout<<"     Default: 0.25"<<std::endl;
		std::cout<<"  -evr <evaporation rate>"<<std::endl;
		std::cout<<"     Water evaporation rate in cm/s"<<std::endl;
		std::cout<<"     Default: 0.0"<<std::endl;
		std::cout<<"  -fpv"<<std::endl;
		std::cout<<"     Fixes the navigation transformation so that Kinect camera and"<<std::endl;
		std::cout<<"     projector are aligned, as defined by the projector calibration file"<<std::endl;
		std::cout<<"  -hs"<<std::endl;
		std::cout<<"     Enables hill shading"<<std::endl;
		std::cout<<"  -us"<<std::endl;
		std::cout<<"     Enables shadows"<<std::endl;
		std::cout<<"  -uhm"<<std::endl;
		std::cout<<"     Enables elevation color mapping"<<std::endl;
		std::cout<<"  -rws"<<std::endl;
		std::cout<<"     Renders water surface as geometric surface"<<std::endl;
		}
	
	/* Enable background USB event handling: */
	usbContext.startEventHandling();
	
	/* Open the Kinect camera device: */
	camera=new Kinect::Camera(usbContext,cameraIndex);
	camera->setCompressDepthFrames(true);
	camera->setSmoothDepthFrames(false);
	for(int i=0;i<2;++i)
		frameSize[i]=camera->getActualFrameSize(Kinect::FrameSource::DEPTH)[i];
	
	/* Get the camera's per-pixel depth correction parameters: */
	Misc::SelfDestructPointer<Kinect::FrameSource::DepthCorrection> depthCorrection(camera->getDepthCorrectionParameters());
	
	/* Get the camera's intrinsic parameters: */
	cameraIps=camera->getIntrinsicParameters();
	
	/* Read the sandbox layout file: */
	Geometry::Plane<double,3> basePlane;
	Geometry::Point<double,3> basePlaneCorners[4];
	{
	IO::ValueSource layoutSource(Vrui::openFile(sandboxLayoutFileName.c_str()));
	layoutSource.skipWs();
	std::string s=layoutSource.readLine();
	basePlane=Misc::ValueCoder<Geometry::Plane<double,3> >::decode(s.c_str(),s.c_str()+s.length());
	basePlane.normalize();
	for(int i=0;i<4;++i)
		{
		layoutSource.skipWs();
		s=layoutSource.readLine();
		basePlaneCorners[i]=Misc::ValueCoder<Geometry::Point<double,3> >::decode(s.c_str(),s.c_str()+s.length());
		}
	}
	
	/* Load the height color map: */
	std::vector<GLColorMap::Color> heightMapColors;
	std::vector<GLdouble> heightMapKeys;
	IO::ValueSource heightMapSource(Vrui::openFile(heightColorMapFileName.c_str()));
	if(Misc::hasCaseExtension(heightColorMapFileName.c_str(),".cpt"))
		{
		heightMapSource.setPunctuation("\n");
		heightMapSource.skipWs();
		while(!heightMapSource.eof())
			{
			/* Read the next color map key value: */
			heightMapKeys.push_back(GLdouble(heightMapSource.readNumber()));
			
			/* Read the next color map color value: */
			GLColorMap::Color color;
			for(int i=0;i<3;++i)
				color[i]=GLColorMap::Color::Scalar(heightMapSource.readNumber()/255.0);
			color[3]=GLColorMap::Color::Scalar(1);
			heightMapColors.push_back(color);
			if(!heightMapSource.isLiteral('\n'))
				Misc::throwStdErr("Sandbox::Sandbox: Format error in color map file %s",heightColorMapFileName.c_str());
			}
		}
	else
		{
		heightMapSource.setPunctuation(",\n");
		heightMapSource.skipWs();
		while(!heightMapSource.eof())
			{
			/* Read the next color map key value: */
			heightMapKeys.push_back(GLdouble(heightMapSource.readNumber()));
			if(!heightMapSource.isLiteral(','))
				Misc::throwStdErr("Sandbox::Sandbox: Format error in color map file %s",heightColorMapFileName.c_str());
			
			/* Read the next color map color value: */
			GLColorMap::Color color;
			for(int i=0;i<3;++i)
				color[i]=GLColorMap::Color::Scalar(heightMapSource.readNumber());
			color[3]=GLColorMap::Color::Scalar(1);
			heightMapColors.push_back(color);
			if(!heightMapSource.isLiteral('\n'))
				Misc::throwStdErr("Sandbox::Sandbox: Format error in color map file %s",heightColorMapFileName.c_str());
			}
		}
	heightMap=GLColorMap(heightMapKeys.size(),&heightMapColors[0],&heightMapKeys[0]);
	
	/* Limit the valid elevation range to the extent of the height color map: */
	if(elevationMin<heightMap.getScalarRangeMin())
		elevationMin=heightMap.getScalarRangeMin();
	if(elevationMax>heightMap.getScalarRangeMax())
		elevationMax=heightMap.getScalarRangeMax();
	
	/* Create the frame filter object: */
	frameFilter=new FrameFilter(frameSize,numAveragingSlots,cameraIps.depthProjection,basePlane);
	frameFilter->setDepthCorrection(*depthCorrection);
	frameFilter->setValidElevationInterval(cameraIps.depthProjection,basePlane,elevationMin,elevationMax);
	frameFilter->setStableParameters(minNumSamples,maxVariance);
	frameFilter->setHysteresis(hysteresis);
	frameFilter->setSpatialFilter(true);
	frameFilter->setOutputFrameFunction(Misc::createFunctionCall(this,&Sandbox::receiveFilteredFrame));
	
	/* Limit the valid rain elevation range to above the valid elevation range: */
	if(rainElevationMin<elevationMax)
		rainElevationMin=elevationMax;
	if(rainElevationMax<rainElevationMin)
		rainElevationMax=rainElevationMin;
	
	/* Create the rain maker object: */
	rainMaker=new RainMaker(frameSize,camera->getActualFrameSize(Kinect::FrameSource::COLOR),cameraIps.depthProjection,cameraIps.colorProjection,basePlane,rainElevationMin,rainElevationMax,20);
	rainMaker->setDepthIsFloat(true);
	rainMaker->setOutputBlobsFunction(Misc::createFunctionCall(this,&Sandbox::receiveRainObjects));
	
	/* Create a second frame filter for the rain maker: */
	rmFrameFilter=new FrameFilter(frameSize,10,cameraIps.depthProjection,basePlane);
	rmFrameFilter->setDepthCorrection(*depthCorrection);
	rmFrameFilter->setValidElevationInterval(cameraIps.depthProjection,basePlane,rainElevationMin,rainElevationMax);
	rmFrameFilter->setStableParameters(5,3);
	rmFrameFilter->setRetainValids(false);
	rmFrameFilter->setInstableValue(2047.0f);
	rmFrameFilter->setSpatialFilter(false);
	rmFrameFilter->setOutputFrameFunction(Misc::createFunctionCall(rainMaker,&RainMaker::receiveRawDepthFrame));
	
	/* Start streaming depth frames: */
	camera->startStreaming(Misc::createFunctionCall(rainMaker,&RainMaker::receiveRawColorFrame),Misc::createFunctionCall(this,&Sandbox::rawDepthFrameDispatcher));
	
	/* Load the projector transformation: */
	if(fixProjectorView)
		{
		std::string transformFileName=CONFIGDIR;
		transformFileName.push_back('/');
		transformFileName.append("ProjectorMatrix.dat");
		try
			{
			IO::FilePtr transformFile=Vrui::openFile(transformFileName.c_str(),IO::File::ReadOnly);
			transformFile->setEndianness(Misc::LittleEndian);
			double pt[16];
			transformFile->read(pt,16);
			projectorTransform=PTransform::fromRowMajor(pt);
			}
		catch(std::runtime_error err)
			{
			std::cerr<<"Cannot fix projector view due to exception "<<err.what()<<std::endl;
			fixProjectorView=false;
			}
		}
	
	/* Calculate a bounding box around all potential surfaces: */
	bbox=Box::empty;
	for(int i=0;i<4;++i)
		{
		bbox.addPoint(basePlane.project(basePlaneCorners[i])+basePlane.getNormal()*elevationMin);
		bbox.addPoint(basePlane.project(basePlaneCorners[i])+basePlane.getNormal()*elevationMax);
		}
	
	/* Initialize the water flow simulator: */
	waterTable=new WaterTable2(wtSize[0],wtSize[1],basePlane,basePlaneCorners);
	waterTable->setElevationRange(elevationMin,rainElevationMax);
	waterTable->setWaterDeposit(evaporationRate);
	
	/* Register a render function with the water table: */
	addWaterFunction=Misc::createFunctionCall(this,&Sandbox::addWater);
	waterTable->addRenderFunction(addWaterFunction);
	addWaterFunctionRegistered=true;
	
	/* Initialize the surface renderer: */
	surfaceRenderer=new SurfaceRenderer(frameSize,cameraIps.depthProjection,basePlane);
	surfaceRenderer->setUseHeightMap(useHeightMap);
	surfaceRenderer->setHeightMapRange(heightMap.getNumEntries(),heightMap.getScalarRangeMin(),heightMap.getScalarRangeMax());
	surfaceRenderer->setDrawContourLines(useContourLines);
	surfaceRenderer->setContourLineDistance(contourLineSpacing);
	if(hillshade)
		surfaceRenderer->setIlluminate(true);
	if(waterTable!=0&&waterSpeed>0.0)
		{
		if(renderWaterSurface)
			{
			/* Create a second surface renderer to render the water surface directly: */
			SurfaceRenderer::PTransform waterTransform(1);
			WaterTable2::Box wd=waterTable->getDomain();
			waterTransform.getMatrix()(0,0)=SurfaceRenderer::Scalar(wd.max[0]-wd.min[0])/SurfaceRenderer::Scalar(wtSize[0]);
			waterTransform.getMatrix()(0,3)=SurfaceRenderer::Scalar(wd.min[0]);
			waterTransform.getMatrix()(1,1)=SurfaceRenderer::Scalar(wd.max[1]-wd.min[1])/SurfaceRenderer::Scalar(wtSize[1]);
			waterTransform.getMatrix()(1,3)=SurfaceRenderer::Scalar(wd.min[1]);
			waterTransform.getMatrix()(2,3)=SurfaceRenderer::Scalar(-0.01);
			waterTransform.leftMultiply(Geometry::invert(waterTable->getBaseTransform()));
			waterRenderer=new SurfaceRenderer(wtSize,waterTransform,basePlane);
			waterRenderer->setUsePreboundDepthTexture(true);
			waterRenderer->setUseHeightMap(false);
			waterRenderer->setDrawContourLines(false);
			waterRenderer->setIlluminate(true);
			}
		else
			{
			surfaceRenderer->setWaterTable(waterTable);
			surfaceRenderer->setAdvectWaterTexture(true);
			surfaceRenderer->setWaterOpacity(waterOpacity);
			}
		}
	
	#if 0
	/* Create a fixed-position light source: */
	sun=Vrui::getLightsourceManager()->createLightsource(true);
	for(int i=0;i<Vrui::getNumViewers();++i)
		Vrui::getViewer(i)->setHeadlightState(false);
	sun->enable();
	sun->getLight().position=GLLight::Position(1,0,1,0);
	#endif
	
	/* Create the main menu: */
	mainMenu=createMainMenu();
	Vrui::setMainMenu(mainMenu);
	
	/* Initialize the navigation transformation: */
	Vrui::Point::AffineCombiner cc;
	for(int i=0;i<4;++i)
		cc.addPoint(Vrui::Point(basePlane.project(basePlaneCorners[i])));
	Vrui::Point c=cc.getPoint();
	Vrui::Scalar maxDist(0);
	for(int i=0;i<4;++i)
		{
		Vrui::Scalar dist=Geometry::dist(Vrui::Point(basePlane.project(basePlaneCorners[i])),c);
		if(maxDist<dist)
			maxDist=dist;
		}
	Vrui::setNavigationTransformation(c,maxDist,Geometry::normal(Vrui::Vector(basePlane.getNormal())));
	}

Sandbox::~Sandbox(void)
	{
	/* Stop streaming depth frames: */
	camera->stopStreaming();
	delete camera;
	delete frameFilter;
	
	/* Delete helper objects: */
	delete surfaceRenderer;
	delete waterTable;
	delete rmFrameFilter;
	delete rainMaker;
	delete addWaterFunction;
	delete waterRenderer;
	
	delete mainMenu;
	}

void Sandbox::frame(void)
	{
	/* Check if the filtered frame has been updated: */
	if(filteredFrames.lockNewValue())
		{
		/* Update the surface renderer's depth image: */
		surfaceRenderer->setDepthImage(filteredFrames.getLockedValue());
		}
	
	/* Lock the most recent rain object list: */
	rainObjects.lockNewValue();
	#if 0
	bool registerWaterFunction=!rainObjects.getLockedValue().empty();
	if(addWaterFunctionRegistered!=registerWaterFunction)
		{
		if(registerWaterFunction)
			waterTable->addRenderFunction(addWaterFunction);
		else
			waterTable->removeRenderFunction(addWaterFunction);
		addWaterFunctionRegistered=registerWaterFunction;
		}
	#endif
	
	/* Update the surface renderer: */
	surfaceRenderer->setAnimationTime(Vrui::getApplicationTime());
	
	if(pauseUpdates)
		Vrui::scheduleUpdate(Vrui::getApplicationTime()+1.0/30.0);
	}

void Sandbox::display(GLContextData& contextData) const
	{
	/* Get the data item: */
	DataItem* dataItem=contextData.retrieveDataItem<DataItem>(this);
	
	if(fixProjectorView)
		{
		/* Install the projector transformation: */
		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glLoadMatrix(projectorTransform);
		glMultMatrix(Geometry::invert(Vrui::getDisplayState(contextData).modelviewNavigational));
		glMatrixMode(GL_MODELVIEW);
		}
	
		/* Render the surface with height map: */
		surfaceRenderer->glRenderSinglePass(dataItem->heightColorMapObject,contextData);

        if(fixProjectorView)
		{
		/* Go back to regular navigation space: */
		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
		glMatrixMode(GL_MODELVIEW);
		}
	}

void Sandbox::initContext(GLContextData& contextData) const
	{
	/* Create a data item and add it to the context: */
	DataItem* dataItem=new DataItem;
	contextData.addDataItem(this,dataItem);
	
	/* Upload the height color map as a 1D texture: */
	glGenTextures(1,&dataItem->heightColorMapObject);
	glBindTexture(GL_TEXTURE_1D,dataItem->heightColorMapObject);
	glTexParameteri(GL_TEXTURE_1D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
	glTexParameteri(GL_TEXTURE_1D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
	glTexParameteri(GL_TEXTURE_1D,GL_TEXTURE_WRAP_S,GL_CLAMP_TO_EDGE);
	glTexImage1D(GL_TEXTURE_1D,0,GL_RGB8,heightMap.getNumEntries(),0,GL_RGBA,GL_FLOAT,heightMap.getColors());
	glBindTexture(GL_TEXTURE_1D,0);
	
	{
	/* Save the currently bound frame buffer: */
	GLint currentFrameBuffer;
	glGetIntegerv(GL_FRAMEBUFFER_BINDING_EXT,&currentFrameBuffer);
	
	/* Set the default shadow buffer size: */
	dataItem->shadowBufferSize[0]=1024;
	dataItem->shadowBufferSize[1]=1024;
	
	/* Generate the shadow rendering frame buffer: */
	glGenFramebuffersEXT(1,&dataItem->shadowFramebufferObject);
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT,dataItem->shadowFramebufferObject);
	
	/* Generate a depth texture for shadow rendering: */
	glGenTextures(1,&dataItem->shadowDepthTextureObject);
	glBindTexture(GL_TEXTURE_2D,dataItem->shadowDepthTextureObject);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_COMPARE_MODE_ARB,GL_COMPARE_R_TO_TEXTURE);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_COMPARE_FUNC_ARB,GL_LEQUAL);
	glTexParameteri(GL_TEXTURE_2D,GL_DEPTH_TEXTURE_MODE_ARB,GL_INTENSITY);
	glTexImage2D(GL_TEXTURE_2D,0,GL_DEPTH_COMPONENT24_ARB,dataItem->shadowBufferSize[0],dataItem->shadowBufferSize[1],0,GL_DEPTH_COMPONENT,GL_UNSIGNED_BYTE,0);
	glBindTexture(GL_TEXTURE_2D,0);
	
	/* Attach the depth texture to the frame buffer object: */
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT,GL_DEPTH_ATTACHMENT_EXT,GL_TEXTURE_2D,dataItem->shadowDepthTextureObject,0);
	glDrawBuffer(GL_NONE);
	glReadBuffer(GL_NONE);
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT,currentFrameBuffer);
	} 
	}

/*************
Main function:
*************/

int main(int argc,char* argv[])
	{
	try
		{
		char** appDefault=0;
		Sandbox app(argc,argv,appDefault);
		app.run();
		}
	catch(std::runtime_error err)
		{
		std::cerr<<"Caught exception "<<err.what()<<std::endl;
		return 1;
		}
	
	return 0;
	}
