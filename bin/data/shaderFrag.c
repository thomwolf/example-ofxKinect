//Specify compiler to use GLSL version 1.2
//Enable working with textures of any dimensions
//Declare texture texture0, which is linked when you use fbo.bind(), or any other texture.bind().

//#version 430 compatibility
#version 120
#extension GL_ARB_texture_rectangle : enable
#extension GL_EXT_gpu_shader4 : enable

uniform sampler2DRect texture0; // kinect grayscale depth image
uniform sampler2DRect texture1; // colormap
uniform float texsize; //colormap size
uniform float contourLineFactor; //contourline factor
uniform float heightMapScale; //heightMapScale factor
uniform float heightMapOffset; //heightMapOffset factor
uniform float baseline; //baseline factor

void main(){
    // Compute colormaping
    vec2 pos = gl_TexCoord[0].xy;
    vec4 color1 =  texture2DRect(texture0, pos);     // depth value
//    float rval =color1.r/1500;
//    vec4 color = vec4(rval,0.0,0.0,1.0);

    float depthvalue = baseline-color1.r; //color1.r get depth value, normally in [0..1]
    float heightColorMapTexCoord = (depthvalue*heightMapScale+heightMapOffset)*texsize;
    vec2 depthPos = vec2(heightColorMapTexCoord, 0.5);//depthvalue*texsize, 0.5);
    vec4 color =  texture2DRect(texture1, depthPos);	//colormap converted depth

    //Compute contour lines
//    float contourLineFactor = 2;
	/* Calculate the contour line interval containing each pixel corner by evaluating the half-pixel offset elevation texture: */
	float corner0=floor(texture2DRect(texture0,vec2(pos.x-0.5,pos.y-0.5)).r*contourLineFactor);
	float corner1=floor(texture2DRect(texture0,vec2(pos.x+0.5,pos.y-0.5)).r*contourLineFactor);
	float corner2=floor(texture2DRect(texture0,vec2(pos.x-0.5,pos.y+0.5)).r*contourLineFactor);
	float corner3=floor(texture2DRect(texture0,vec2(pos.x+0.5,pos.y+0.5)).r*contourLineFactor);

	/* Find all pixel edges that cross at least one contour line: */
	int edgeMask=0;
	int numEdges=0;
	if(corner0!=corner1)
    {
		edgeMask+=1;
		++numEdges;
    }
	if(corner2!=corner3)
    {
		edgeMask+=2;
		++numEdges;
    }
	if(corner0!=corner2)
    {
		edgeMask+=4;
		++numEdges;
    }
	if(corner1!=corner3)
    {
		edgeMask+=8;
		++numEdges;
    }

	/* Check for all cases in which the pixel should be colored as a topographic contour line: */
	if(numEdges>2||edgeMask==3||edgeMask==12||(numEdges==2&&mod(floor(pos.x)+floor(pos.y),2.0)==0.0))
    {
		/* Topographic contour lines are rendered in black: */
		color=vec4(0.0,0.0,0.0,1.0);
    }
    
    //Output of shader
    gl_FragColor = color;
}

//uniform sampler2DRect pixelCornerElevationSampler;
//uniform float contourLineFactor;

//void addContourLines(in vec2 fragCoord,inout vec4 baseColor)
//{
//#if 0
//
//	/*********************************************************************
//     Simple algorithm: draws contour line if the area of a pixel crosses at
//     least one contour line. Always draws 4-connected lines.
//     *********************************************************************/
//	
//	/* Calculate the elevation of each pixel corner by evaluating the half-pixel offset elevation texture: */
//	float corner0=texture2DRect(pixelCornerElevationSampler,vec2(fragCoord.x,fragCoord.y)).r;
//	float corner1=texture2DRect(pixelCornerElevationSampler,vec2(fragCoord.x+1.0,fragCoord.y)).r;
//	float corner2=texture2DRect(pixelCornerElevationSampler,vec2(fragCoord.x,fragCoord.y+1.0)).r;
//	float corner3=texture2DRect(pixelCornerElevationSampler,vec2(fragCoord.x+1.0,fragCoord.y+1.0)).r;
//	
//	/* Calculate the elevation range of the pixel's area: */
//	float elMin=min(min(corner0,corner1),min(corner2,corner3));
//	float elMax=max(max(corner0,corner1),max(corner2,corner3));
//	
//	/* Check if the pixel's area crosses at least one contour line: */
//	if(floor(elMin*contourLineFactor)!=floor(elMax*contourLineFactor))
//    {
//		/* Topographic contour lines are rendered in black: */
//		baseColor=vec4(0.0,0.0,0.0,1.0);
//    }
//	
//#else
//	
//	/*********************************************************************
//     More complicated algorithm: draws thinnest possible contour lines by
//     removing redundant 4-connected pixels.
//     *********************************************************************/
//	
//	/* Calculate the contour line interval containing each pixel corner by evaluating the half-pixel offset elevation texture: */
//	float corner0=floor(texture2DRect(pixelCornerElevationSampler,vec2(fragCoord.x,fragCoord.y)).r*contourLineFactor);
//	float corner1=floor(texture2DRect(pixelCornerElevationSampler,vec2(fragCoord.x+1.0,fragCoord.y)).r*contourLineFactor);
//	float corner2=floor(texture2DRect(pixelCornerElevationSampler,vec2(fragCoord.x,fragCoord.y+1.0)).r*contourLineFactor);
//	float corner3=floor(texture2DRect(pixelCornerElevationSampler,vec2(fragCoord.x+1.0,fragCoord.y+1.0)).r*contourLineFactor);
//	
//	/* Find all pixel edges that cross at least one contour line: */
//	int edgeMask=0;
//	int numEdges=0;
//	if(corner0!=corner1)
//    {
//		edgeMask+=1;
//		++numEdges;
//    }
//	if(corner2!=corner3)
//    {
//		edgeMask+=2;
//		++numEdges;
//    }
//	if(corner0!=corner2)
//    {
//		edgeMask+=4;
//		++numEdges;
//    }
//	if(corner1!=corner3)
//    {
//		edgeMask+=8;
//		++numEdges;
//    }
//	
//	/* Check for all cases in which the pixel should be colored as a topographic contour line: */
//	if(numEdges>2||edgeMask==3||edgeMask==12||(numEdges==2&&mod(floor(fragCoord.x)+floor(fragCoord.y),2.0)==0.0))
//    {
//		/* Topographic contour lines are rendered in black: */
//		baseColor=vec4(0.0,0.0,0.0,1.0);
//    }
//	
//#endif
//}


