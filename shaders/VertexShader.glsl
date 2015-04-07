uniform mat4 World;
uniform mat4 View;
uniform mat4 Projection;
varying vec3 position;
varying vec3 color;
varying vec4 cs_position;
varying vec4 cs_color;

void main()
{
    cs_position =  View * World * vec4(position.x, position.y, position.z, 1.0);
    gl_Position = Projection * cs_position;
    cs_color = vec4(color.x, color.y, color.z, 1.0);
}
