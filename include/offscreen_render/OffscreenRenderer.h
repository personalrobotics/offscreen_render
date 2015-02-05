#ifndef OFFSCREENRENDERER_H_
#define OFFSCREENRENDERER_H_

#include "Shader.h"
#include "VertexBuffer.h"
#include <map>
#include <vector>

namespace offscreen_render
{

    typedef GLuint FrameBufferID;
    typedef GLuint TextureID;
    struct Model
    {
            VertexBuffer* buffer;
            Mat4x4 transform;
    };

    class FrameBuffer
    {
        public:
            FrameBuffer();
            virtual ~FrameBuffer();
            void Initialize(int width, int height);
            void Begin();
            void End();
            std::vector<float> AllocateData();
            void CopyData(std::vector<float>& data);
            TextureID textureID;
            TextureID depthID;
            FrameBufferID fboID;
            int width;
            int height;
            std::vector<float> data;
    };

    class OffscreenRenderer
    {
        public:
            OffscreenRenderer();
            virtual ~OffscreenRenderer();

            void Draw();

            void Initialize(int width, int height);

            Shader* shader;
            FrameBuffer buffer;
            std::vector<Model> models;
            Mat4x4 projectionMatrix;
            Mat4x4 viewMatrix;
    };

}

#endif // OFFSCREENRENDERER_H_ 
