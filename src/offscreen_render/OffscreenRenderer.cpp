#include <offscreen_render/OffscreenRenderer.h>

namespace offscreen_render
{

    OffscreenRenderer::OffscreenRenderer()
    {
        shader = 0x0;
    }

    OffscreenRenderer::~OffscreenRenderer()
    {

    }

    void OffscreenRenderer::Draw()
    {
        buffer.Begin();
        shader->Begin();
        shader->SetProjectionMatrix(projectionMatrix);
        shader->SetViewMatrix(viewMatrix);
        for (const Model& model : models)
        {
            shader->SetWorldMatrix(Mat4x4(model.transform.transpose()));
            model.buffer->Begin();
            model.buffer->Draw();
            model.buffer->End();
        }
        shader->End();
        buffer.End();
    }

    void OffscreenRenderer::Initialize(int width, int height)
    {
        buffer.Initialize(width, height);
    }

    std::vector<float> FrameBuffer::AllocateData()
    {
        std::vector<float> toReturn;
        toReturn.resize(width * height * 3);
        return toReturn;
    }
    void FrameBuffer::CopyData(std::vector<float>& data)
    {
        glReadPixels(0, 0, width, height, GL_RGB, GL_FLOAT, data.data());
    }

    FrameBuffer::FrameBuffer()
    {
        fboID = 0;
        textureID = 0;
        depthID = 0;
        width = 0;
        height = 0;
    }

    FrameBuffer::~FrameBuffer()
    {
        glDeleteFramebuffers(1, &fboID);
        glDeleteTextures(1, &textureID);
        glDeleteRenderbuffers(1, &depthID);
    }

    void FrameBuffer::Initialize(int width, int height)
    {
        this->width = width;
        this->height = height;
        fboID = 0;
        glGenFramebuffers(1, &fboID);
        glBindFramebuffer(GL_FRAMEBUFFER, fboID);

        // The texture we're going to render to
        textureID = 0;
        glGenTextures(1, &textureID);

        // "Bind" the newly created texture : all future texture functions will modify this texture
        glBindTexture(GL_TEXTURE_2D, textureID);

        // Give an empty image to OpenGL ( the last "0" )
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32F, width, height, 0, GL_RGB, GL_FLOAT, 0);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

        depthID = 0;
        glGenRenderbuffers(1, &depthID);
        glBindRenderbuffer(GL_RENDERBUFFER, depthID);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, width, height);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depthID);

        // Set "renderedTexture" as our colour attachement #0
        glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, textureID, 0);

        data = AllocateData();

    }

    void FrameBuffer::Begin()
    {
        // Set the list of draw buffers.
        GLenum drawBuffers[1] = {GL_COLOR_ATTACHMENT0};
        glDrawBuffers(1, drawBuffers); // "1" is the size of DrawBuffers
        // Render to our framebuffer
        glBindFramebuffer(GL_FRAMEBUFFER, fboID);
        glViewport(0, 0, width, height);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    }

    void FrameBuffer::End()
    {
        CopyData(data);
    }

}
