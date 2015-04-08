#include <offscreen_render/VertexBuffer.h>

namespace offscreen_render
{

    VertexBuffer::VertexBuffer()
    {
        initialized = false;
    }

    VertexBuffer::~VertexBuffer()
    {
        if(initialized)
        {
            glDeleteBuffers(1, &vertexArrayID);
            glDeleteBuffers(1, &positionID);
            glDeleteBuffers(1, &colorID);
            glDeleteBuffers(1, &indexID);
        }
    }


    void  VertexBuffer::Initialize(const Shader& shader)
    {
        glGenVertexArrays(1, &vertexArrayID);
        glBindVertexArray(vertexArrayID);

        glGenBuffers(1, &positionID);
        glBindBuffer(GL_ARRAY_BUFFER, positionID);
        glBufferData(GL_ARRAY_BUFFER, position_data.size() * sizeof(position_data.data()), position_data.data(), GL_STATIC_DRAW);

        glGenBuffers(1, &colorID);
        glBindBuffer(GL_ARRAY_BUFFER, colorID);
        glBufferData(GL_ARRAY_BUFFER, color_data.size() * sizeof(color_data.data()), color_data.data(), GL_STATIC_DRAW);


        glGenBuffers(1, &indexID);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexID);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, index_data.size() * sizeof(unsigned short), index_data.data(), GL_STATIC_DRAW);
        initialized = true;

        positionAttributeID = 0;
        colorAttributeID = 1;
        glBindAttribLocation(shader.programID, positionAttributeID, "position");
        glBindAttribLocation(shader.programID, colorAttributeID, "color");

    }

    void VertexBuffer::Begin()
    {
        // 1rst attribute buffer : vertices
        glEnableVertexAttribArray(positionAttributeID);
        glBindBuffer(GL_ARRAY_BUFFER, positionID);
        glVertexAttribPointer(
            positionAttributeID,                  // attribute. No particular reason for 0, but must match the layout in the shader.
            3,                  // size
            GL_FLOAT,           // type
            GL_FALSE,           // normalized?
            0,                  // stride
            (void*)0            // array buffer offset
        );

        // 2nd attribute buffer : colors
        glEnableVertexAttribArray(colorAttributeID);
        glBindBuffer(GL_ARRAY_BUFFER, colorID);
        glVertexAttribPointer(
            colorAttributeID,                                // attribute. No particular reason for 1, but must match the layout in the shader.
            3,                                // size
            GL_FLOAT,                         // type
            GL_FALSE,                         // normalized?
            0,                                // stride
            (void*)0                          // array buffer offset
        );

        // Index buffer
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexID);

    }

    void VertexBuffer::Draw()
    {
        // Draw the triangles !
        glDrawElements(
            GL_TRIANGLES,      // mode
            index_data.size(),    // count
            GL_UNSIGNED_SHORT,   // type
            (void*)0           // element array buffer offset
        );

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
