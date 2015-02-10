varying vec4 cs_position;
varying vec4 cs_color;
void main()
{
    //gl_FragColor = vec4(cs_position.x, -cs_position.y, -cs_position.z, 1.0);
    gl_FragColor =vec4(cs_position.x, -cs_position.y, -cs_position.z, 1.0);
}
