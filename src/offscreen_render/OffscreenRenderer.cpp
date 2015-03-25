#include <offscreen_render/OffscreenRenderer.h>

namespace offscreen_render
{
    OffscreenRenderer::OffscreenRenderer()
    {
        depthShader = 0x0;
        colorShader = 0x0;
    }

    OffscreenRenderer::~OffscreenRenderer()
    {

    }

    void OffscreenRenderer::Draw()
    {
        DrawToBuffer(depthShader, &depthBuffer);
        DrawToBuffer(colorShader, &colorBuffer);
    }

    void OffscreenRenderer::Initialize(int width, int height)
    {
        depthBuffer.Initialize(width, height);
        colorBuffer.Initialize(width, height);
    }

}
