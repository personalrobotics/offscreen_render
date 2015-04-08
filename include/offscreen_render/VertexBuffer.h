#ifndef VERTEXBUFFER_H_
#define VERTEXBUFFER_H_

#include "Shader.h"
#include <vector>
#include <GL/glew.h>

namespace offscreen_render
{
    typedef GLuint BufferID;
    typedef GLuint AttribID;

    class VertexBuffer
    {
        public:
            VertexBuffer();
            virtual ~VertexBuffer();

            void Initialize(const Shader& shader);
            void Begin();
            void Draw();
            void End();
            void DebugDraw();

            std::vector<GLfloat> position_data;
            std::vector<GLfloat> color_data;
            std::vector<unsigned short> index_data;

            BufferID positionID;
            BufferID colorID;
            AttribID positionAttributeID;
            AttribID colorAttributeID;
            BufferID indexID;
            BufferID vertexArrayID;
            bool initialized;
    };

}

#endif // VERTEXBUFFER_H_ 
