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

void main(){
    vec2 pos = gl_TexCoord[0].xy;
    vec4 color1 =  texture2DRect(texture0, pos);     // depth value
    float depthvalue = color1.r; //get depth value, normally in [0..1]
    vec2 depthPos = vec2(depthvalue*texsize, 0.5);
    vec4 color =  texture2DRect(texture1, depthPos);	//colormap converted depth

    //Output of shader
    gl_FragColor = color;
}

