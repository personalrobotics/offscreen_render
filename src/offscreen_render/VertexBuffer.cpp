#include <offscreen_render/VertexBuffer.h>

namespace offscreen_render
{

    VertexBuffer::VertexBuffer()
    {
        initialized = false;
    }

    VertexBuffer::~VertexBuffer()
    {
        if (initialized)
        {
            glDeleteBuffers(1, &vertexArrayID);
            glDeleteBuffers(1, &positionID);
            glDeleteBuffers(1, &colorID);
            glDeleteBuffers(1, &indexID);
        }
    }

    void VertexBuffer::Initialize(const Shader& shader)
    {
        glGenVertexArrays(1, &vertexArrayID);
        checkError("glGenVertexArrays");
        glBindVertexArray(vertexArrayID);
        checkError("glBindVertexArray");

        glGenBuffers(1, &positionID);
        checkError("glGenBuffers");
        glBindBuffer(GL_ARRAY_BUFFER, positionID);
        checkError("glBindBuffer");
        glBufferData(GL_ARRAY_BUFFER, position_data.size() * sizeof(float), position_data.data(), GL_STATIC_DRAW);
        checkError("glBufferData");

        glGenBuffers(1, &colorID);
        checkError("glGenBuffers");
        glBindBuffer(GL_ARRAY_BUFFER, colorID);
        checkError("glBindBuffer");
        glBufferData(GL_ARRAY_BUFFER, color_data.size() * sizeof(float), color_data.data(), GL_STATIC_DRAW);
        checkError("glBufferData");

        glGenBuffers(1, &indexID);
        checkError("glGenBuffers");
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexID);
        checkError("glBindBuffer");
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, index_data.size() * sizeof(unsigned short), index_data.data(), GL_STATIC_DRAW);
        checkError("glBufferData");
        initialized = true;

        positionAttributeID = 0;
        colorAttributeID = 1;

    }

    void VertexBuffer::Begin()
    {
        glBindVertexArray(vertexArrayID);
        checkError("glBindVertexArray");
        // 1rst attribute buffer : vertices
        glEnableVertexAttribArray(positionAttributeID);
        checkError("glEnableVertexAttribArray");
        glBindBuffer(GL_ARRAY_BUFFER, positionID);
        checkError("glBindBuffer");
        glVertexAttribPointer(positionAttributeID,                  // attribute. No particular reason for 0, but must match the layout in the shader.
                3,                  // size
                GL_FLOAT,           // type
                GL_FALSE,           // normalized?
                0,                  // stride
                (void*) 0            // array buffer offset
                );
        checkError("glVertexAttribPointer");

        // 2nd attribute buffer : colors
        glEnableVertexAttribArray(colorAttributeID);
        checkError("glEnableVertexAttribArray");
        glBindBuffer(GL_ARRAY_BUFFER, colorID);
        checkError("glBindBuffer");
        glVertexAttribPointer(colorAttributeID,                                // attribute. No particular reason for 1, but must match the layout in the shader.
                3,                                // size
                GL_FLOAT,                         // type
                GL_FALSE,                         // normalized?
                0,                                // stride
                (void*) 0                          // array buffer offset
                );
        checkError("glVertexAttribPointer");
        // Index buffer
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexID);
        checkError("glBindBuffer");

    }

    void VertexBuffer::Draw()
    {
        // Draw the triangles !
        glDrawElements(
        GL_TRIANGLES,      // mode
                index_data.size(),    // count
                GL_UNSIGNED_SHORT,   // type
                (void*) 0           // element array buffer offset
                );
        checkError("glDrawElements");

    }

    void VertexBuffer::DebugDraw()
    {
        glDisableClientState(GL_COLOR_ARRAY);
        glDisableClientState(GL_VERTEX_ARRAY);
        glBegin(GL_TRIANGLES);
        for (size_t i = 0; i < index_data.size(); i++)
        {
            unsigned short idx = index_data.at(i);
            glVertex3f(position_data[idx * 3 + 0], position_data[idx * 3 + 1], position_data[idx * 3 + 2]);
            glColor3f(color_data[idx * 3 + 0], color_data[idx * 3 + 1], color_data[idx * 3 + 2]);
        }
        glEnd();
    }

    void VertexBuffer::End()
    {
        glDisableVertexAttribArray(positionAttributeID);
        glDisableVertexAttribArray(colorAttributeID);
    }

}
