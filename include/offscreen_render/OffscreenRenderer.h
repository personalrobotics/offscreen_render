#ifndef OFFSCREENRENDERER_H_
#define OFFSCREENRENDERER_H_

#include "Shader.h"
#include "VertexBuffer.h"
#include <map>
#include <vector>
#include <iostream>

namespace offscreen_render
{

    typedef GLuint FrameBufferID;
    typedef GLuint TextureID;
    struct Model
    {
            VertexBuffer* buffer;
            Mat4x4 transform;
    };

    template<typename DataType, int NumChannels> class FrameBuffer
    {
        public:
            void checkError(const char *str)
            {
                GLenum error;
                if ((error = glGetError()) != GL_NO_ERROR)
                {
                    fprintf(stderr, "GL Error: %s (%s)\n", gluErrorString(error), str);
                    throw -1;
                }
            }

            void checkFramebufferError(GLenum status)
            {
                if (status != GL_FRAMEBUFFER_COMPLETE)
                {
                    switch (status)
                    {
                        fprintf(stderr,"Something wrong with framebuffer!\n");
                    case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT:
                        fprintf(stderr,"An attachment could not be bound to frame buffer object!");
                        break;

                    case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT:
                        fprintf(stderr,"Attachments are missing! At least one image (texture) must be bound to the frame buffer object!");
                        break;

                    case GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS_EXT:
                        fprintf(stderr,"The dimensions of the buffers attached to the currently used frame buffer object do not match!");
                        break;

                    case GL_FRAMEBUFFER_INCOMPLETE_FORMATS_EXT:
                        fprintf(stderr,"The formats of the currently used frame buffer object are not supported or do not fit together!");
                        break;

                    case GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER:
                        fprintf(stderr,"A Draw buffer is incomplete or undefinied. All draw buffers must specify attachment points that have images attached.");
                        break;

                    case GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER:
                        fprintf(stderr,"A Read buffer is incomplete or undefinied. All read buffers must specify attachment points that have images attached.");
                        break;

                    case GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE:
                        fprintf(stderr,"All images must have the same number of multisample samples.");
                        break;

                    case GL_FRAMEBUFFER_INCOMPLETE_LAYER_TARGETS:
                        fprintf(stderr,
                                "If a layered image is attached to one attachment, then all attachments must be layered attachments. The attached layers do not have to have the same number of layers, nor do the layers have to come from the same kind of texture.");
                        break;

                    case GL_FRAMEBUFFER_UNSUPPORTED:
                        fprintf(stderr,"Attempt to use an unsupported format combinaton!");
                        break;

                    default:
                        fprintf(stderr,"Unknown error while attempting to create frame buffer object! Error: %u ID: %u", status, fboID);
                        break;
                    }
                    fprintf(stderr,"\n");
                    throw -1;
                }
            }

            void CopyData(std::vector<float>& data)
            {
                glBindFramebuffer(GL_FRAMEBUFFER, fboID);
                checkError("glBindFramebuffer");
                glReadBuffer(GL_COLOR_ATTACHMENT0);
                checkError("glReadBuffer");
                glReadPixels(0, 0, width, height, GL_RGB, GL_FLOAT, data.data());
                checkError("glReadPixels");
                //glGetTexImage(GL_TEXTURE_2D, 0,  GL_RGB, GL_FLOAT, data.data());
            }

            void CopyData(std::vector<uint8_t>& data)
            {
                glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fboID);
                checkError("glBindFramebuffer");
                glGetTexImage(GL_TEXTURE_2D, 0, GL_RGB, GL_UNSIGNED_BYTE, data.data());
                checkError("glGetTexImage");
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
                checkError("glDeleteFramebuffers");
                glDeleteTextures(1, &textureID);
                checkError("glDeleteTextures");
                glDeleteRenderbuffers(1, &depthID);
                checkError("glDeleteRenderbuffers");
            }

            GLboolean QueryExtension(const char *extName)
            {
                /*
                ** Search for extName in the extensions string. Use of strstr()
                ** is not sufficient because extension names can be prefixes of
                ** other extension names. Could use strtok() but the constant
                ** string returned by glGetString might be in read-only memory.
                */
                char *p;
                char *end;
                int extNameLen;

                extNameLen = strlen(extName);

                p = (char *)glGetString(GL_EXTENSIONS);
                if (NULL == p) {
                    return GL_FALSE;
                }

                end = p + strlen(p);

                while (p < end) {
                    int n = strcspn(p, " ");
                    if ((extNameLen == n) && (strncmp(extName, p, n) == 0)) {
                        return GL_TRUE;
                    }
                    p += (n + 1);
                }
                return GL_FALSE;
            }

            void Initialize(int width, int height)
            {
                this->width = width;
                this->height = height;


                fboID = 0;
                glGenFramebuffers(1, &fboID);
                checkError("glGenFramebuffers");
                glBindFramebuffer(GL_FRAMEBUFFER, fboID);
                checkError("glBindFramebuffer");

                // The texture we're going to render to
                textureID = 0;
                glGenTextures(1, &textureID);
                checkError("glGenTextures");

                // "Bind" the newly created texture : all future texture functions will modify this texture
                glBindTexture(GL_TEXTURE_2D, textureID);
                checkError("glBindTexture");

                // Give an empty image to OpenGL ( the last "0" )
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, width, height, 0, GL_RGBA, GL_FLOAT, 0);
                checkError("glTexImage");

                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
                checkError("glTexParameteri");
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
                checkError("glTexParameteri");
                depthID = 0;

                // Set "renderedTexture" as our colour attachement #0
                glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, textureID, 0);
                checkError("glFramebufferTexture");

                printf("Bound texture\n");
                GLenum status = glCheckFramebufferStatusEXT(GL_FRAMEBUFFER);
                checkError("glCheckFramebufferStatus");
                checkFramebufferError(status);

                glGenRenderbuffers(1, &depthID);
                checkError("glGenRenderbuffers");
                glBindRenderbuffer(GL_RENDERBUFFER, depthID);
                checkError("glBindRenderbuffer");
                glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, width, height);
                checkError("glRenderbufferStorage");
                glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depthID);
                checkError("glFramebufferRenderbuffer");

                printf("Bound depth buffer\n");

                status = glCheckFramebufferStatusEXT(GL_FRAMEBUFFER);
                checkError("glCheckFramebufferStatus");
                checkFramebufferError(status);

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
                GLenum drawBuffers[1] =
                { GL_COLOR_ATTACHMENT0 };
                glDrawBuffers(1, drawBuffers); // "1" is the size of DrawBuffers
                checkError("DrawBuffers");
                // Render to our framebuffer
                glBindFramebufferEXT(GL_FRAMEBUFFER, fboID);
                checkError("BindFrameBuffer");

                /*
                glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, textureID, 0);
                checkError("glFramebufferTexture2D");

                glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depthID);
                checkError("glFramebufferRenderbuffer");

                // Always check that our framebuffer is ok
                GLenum status = glCheckFramebufferStatusEXT(GL_FRAMEBUFFER);
                checkError("glCheckFramebufferStatus");
                checkFramebufferError(status);
                */
                glViewport(0, 0, width, height);
                glClearColor(0, 0, 0, 1.0);
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                checkError("glClear");
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
            enum RenderMode
            {
                Onscreen,
                Offscreen
            };

            OffscreenRenderer();
            virtual ~OffscreenRenderer();

            void Draw();

            template<typename T, int N> void DrawToBuffer(Shader* shader, FrameBuffer<T, N>* buffer)
            {
                buffer->Begin();
                {
                    Draw(shader);
                }
                buffer->End();
            }

            void Draw(Shader* shader)
            {
		
                shader->Begin();
                {
                    shader->SetProjectionMatrix(projectionMatrix);
                    shader->SetViewMatrix(viewMatrix);
                    for (size_t i = 0; i < models.size(); i++)
                    {
                        const Model& model = models.at(i);
                        shader->SetWorldMatrix(Mat4x4(model.transform));

                        model.buffer->Begin();
                        {
                            model.buffer->Draw();
                        }
                        model.buffer->End();
                    }
                }
                shader->End();

            }

            void Initialize(int width, int height);

            Shader* colorShader;
            Shader* depthShader;
            FrameBuffer<float, 3> colorBuffer;
            FrameBuffer<float, 3> depthBuffer;
            std::vector<Model> models;
            Mat4x4 projectionMatrix;
            Mat4x4 viewMatrix;
            RenderMode renderMode;
    };

}

#endif // OFFSCREENRENDERER_H_ 
