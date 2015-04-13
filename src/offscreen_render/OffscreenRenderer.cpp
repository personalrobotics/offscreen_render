#include <offscreen_render/OffscreenRenderer.h>

namespace offscreen_render
{
    OffscreenRenderer::OffscreenRenderer()
    {
        depthShader = 0x0;
        colorShader = 0x0;
        renderMode = OffscreenRenderer::Offscreen;
    }

    OffscreenRenderer::~OffscreenRenderer()
    {

    }

    void OffscreenRenderer::Draw()
    {
        switch(renderMode)
        {
            case Offscreen:
                //DrawToBuffer(depthShader, &depthBuffer);
                DrawToBuffer(colorShader, &colorBuffer);
                break;
            case Onscreen:
                glBindFramebuffer(GL_FRAMEBUFFER, 0);
                Draw(colorShader);
                break;
        }
    }

    void OffscreenRenderer::Initialize(int width, int height)
    {
        depthBuffer.Initialize(width, height);
        colorBuffer.Initialize(width, height);
    }

}
