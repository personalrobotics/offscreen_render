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

    template <typename DataType, int NumChannels> class FrameBuffer
    {
        public:
            void CopyData(std::vector<float>& data)
            {
                glReadPixels(0, 0, width, height, GL_RGB, GL_FLOAT, data.data());
            }

            void CopyData(std::vector<uint8_t>& data)
            {
                glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, data.data());
            }

            FrameBuffer()
            {
                fboID = 0;
                textureID = 0;
                depthID = 0;
                width = 0;
                height = 0;
            }

            ~FrameBuffer()
            {
                glDeleteFramebuffers(1, &fboID);
                glDeleteTextures(1, &textureID);
                glDeleteRenderbuffers(1, &depthID);
            }

            void Initialize(int width, int height)
            {
                printf("Initializing a buffer\n");
                this->width = width;
                this->height = height;

                printf("Gen frame buffers\n");
                fboID = 0;
                glGenFramebuffers(1, &fboID);
                glBindFramebuffer(GL_FRAMEBUFFER, fboID);

                printf("Gen textures\n");
                // The texture we're going to render to
                textureID = 0;
                glGenTextures(1, &textureID);

                // "Bind" the newly created texture : all future texture functions will modify this texture
                glBindTexture(GL_TEXTURE_2D, textureID);

                // Give an empty image to OpenGL ( the last "0" )
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32F, width, height, 0, GL_RGB, GL_FLOAT, 0);

                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

                printf("Gen depthbuffer\n");
                depthID = 0;

                printf("Gen render buffer\n");
                glGenRenderbuffers(1, &depthID);
                printf("Bind render buffer %d\n", depthID);
                glBindRenderbuffer(GL_RENDERBUFFER, depthID);
                printf("Render buffer storage\n");
                glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, width, height);
                printf("Frame buffer renderbuffer\n");
                glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depthID);

                printf("Framebuffer texture %d.\n", textureID);
                // Set "renderedTexture" as our colour attachement #0
                glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, textureID, 0);

                printf("Allocating.\n");
                data = AllocateData();

            }

            std::vector<DataType> AllocateData()
            {
                std::vector<DataType> toReturn;
                toReturn.resize(width * height * NumChannels);
                return toReturn;
            }


            void Begin()
            {
                // Set the list of draw buffers.
                GLenum drawBuffers[1] = {GL_COLOR_ATTACHMENT0};
                glDrawBuffers(1, drawBuffers); // "1" is the size of DrawBuffers
                // Render to our framebuffer
                glBindFramebuffer(GL_FRAMEBUFFER, fboID);
                glViewport(0, 0, width, height);
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            }

            void End()
            {
                CopyData(data);
            }
            TextureID textureID;
            TextureID depthID;
            FrameBufferID fboID;
            int width;
            int height;
            std::vector<DataType> data;
    };

    class OffscreenRenderer
    {
        public:
            OffscreenRenderer();
            virtual ~OffscreenRenderer();

            void Draw();

            template <typename T, int N> void DrawToBuffer(Shader* shader, FrameBuffer<T, N>* buffer)
            {
                //buffer->Begin();
                {
                    shader->Begin();
                    {
                        shader->SetProjectionMatrix(projectionMatrix);
                        shader->SetViewMatrix(viewMatrix);
                        for (const Model& model : models)
                        {
                            shader->SetWorldMatrix(Mat4x4(model.transform.transpose()));
                            model.buffer->Begin();
                            {
                                model.buffer->Draw();
                            }
                            model.buffer->End();
                        }
                    }
                    shader->End();
                }
                //buffer->End();
            }

            void Initialize(int width, int height);

            Shader* colorShader;
            Shader* depthShader;
            FrameBuffer<uint8_t, 3> colorBuffer;
            FrameBuffer<float, 3> depthBuffer;
            std::vector<Model> models;
            Mat4x4 projectionMatrix;
            Mat4x4 viewMatrix;
    };

}

#endif // OFFSCREENRENDERER_H_ 
