#ifndef SHADER_H_
#define SHADER_H_

#include "Geometry.h"
#include <string>
#include <GL/glew.h>
#include <stdio.h>

namespace offscreen_render
{
    typedef GLuint ProgramID;
    typedef GLuint ParamID;


    class Shader
    {
        public:
            void checkError(const char *str)
            {
                GLenum error;

                if ((error = glGetError()) != GL_NO_ERROR)
                    printf("GL Error: %s (%s)\n", gluErrorString(error), str);
            }

            Shader();
            virtual ~Shader();


            inline void Begin()
            {
                // Use our shader
                glUseProgram(programID);
                checkError("glUseProgram");
            }

            inline void End()
            {
                //
            }

            inline void  SetMatrixParam(const ParamID& param, const Mat4x4& matrix)
            {
                glUniformMatrix4fv(param, 1, GL_FALSE, matrix.data());
                checkError("glUniformMatrix4fv");
            }

            inline void SetProjectionMatrix(const Mat4x4& matrix)
            {
                SetMatrixParam(projectionMatrixID, matrix);
            }

            inline void SetViewMatrix(const Mat4x4& matrix)
            {
                SetMatrixParam(viewMatrixID, matrix);
            }

            inline void SetWorldMatrix(const Mat4x4& matrix)
            {
                SetMatrixParam(worldMatrixID, matrix);
            }


            bool LoadFromFile(const std::string& fragmentFile, const std::string& vertexFile);

            ProgramID programID;

            ParamID projectionMatrixID;
            ParamID viewMatrixID;
            ParamID worldMatrixID;
    };

}

#endif // SHADER_H_ 
