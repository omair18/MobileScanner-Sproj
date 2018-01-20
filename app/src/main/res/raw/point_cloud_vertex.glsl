precision mediump float;

attribute vec4 a_Position;
uniform mat4 u_MVP;
uniform float u_PointThickness;
attribute vec4 u_PointColor;
varying vec4 v_PointColor;

/*
Dmitry Brant, 2017
*/
void main() {
    gl_Position = u_MVP * a_Position;
    gl_PointSize = u_PointThickness;
    //gl_Color = u_PointColor;
    //gl_Color = vec3(1.0, 0.0, 0.0);
    v_PointColor = u_PointColor;
}
