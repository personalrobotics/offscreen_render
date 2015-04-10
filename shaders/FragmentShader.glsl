#version 130
in vec4 cs_position;
in vec4 cs_color;
out vec4 fragColor;
void main()
{
    //gl_FragColor = vec4(cs_position.x, -cs_position.y, -cs_position.z, 1.0);
    fragColor =vec4(cs_position.x, -cs_position.y, -cs_position.z, 1.0);

}
