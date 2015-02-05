varying vec4 cs_position;
void main()
{
    gl_FragColor = vec4(cs_position.x, -cs_position.y, -cs_position.z, 1.0);
}