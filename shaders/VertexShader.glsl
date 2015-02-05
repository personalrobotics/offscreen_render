uniform mat4 World;
uniform mat4 View;
uniform mat4 Projection;
attribute vec3 position;
attribute vec3 color;
varying vec4 cs_position;

void main()
{
    cs_position =  View * World * vec4(position, 1.0);
    gl_Position = Projection * cs_position;
}