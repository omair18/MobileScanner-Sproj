precision mediump float;

uniform vec3 u_ambientColor;
//attribute vec3 u_ambientColor;
varying vec4 v_PointColor;
/*
Dmitry Brant, 2017
*/
void main()
{
    float maxDepth = 50.0;
    float depth = gl_FragCoord.z / gl_FragCoord.w;
    //gl_FragColor = vec4(u_ambientColor * (maxDepth / depth), 1.0);
    //gl_FragColor = vec4(u_ambientColor, 1.0); //  vec4(1.0, 0.0, 0.0, 1.0);
    //glColor3f(1.0, 1.0, 1.0);
   // vec4(u_ambientColor * (maxDepth / depth), 1.0);
   gl_FragColor = v_PointColor;
}
