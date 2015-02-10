#include <stdio.h>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <stdlib.h>
#include <string.h>
#include <GL/glew.h>

#include <offscreen_render/Shader.h>
using namespace std;

namespace offscreen_render
{

    Shader::Shader()
    {

    }

    Shader::~Shader()
    {
        if(programID)
            glDeleteProgram(programID);
    }

    bool Shader::LoadFromFile(const std::string& fragmentFile, const std::string& vertexFile)
    {
        printf("Creating shaders...\n");
        // Create the shaders
        ProgramID vertexShaderID = glCreateShader(GL_VERTEX_SHADER);
        ProgramID fragmentShaderID = glCreateShader(GL_FRAGMENT_SHADER);

        printf("Opening vertex shader file...\n");
        // Read the Vertex Shader code from the file
        std::string vertexShaderCode;
        std::ifstream vertexShaderStream(vertexFile.c_str(), std::ios::in);
        if (vertexShaderStream.is_open())
        {
            std::string Line = "";
            while (getline(vertexShaderStream, Line))
                vertexShaderCode += "\n" + Line;
            vertexShaderStream.close();
        }
        else
        {
            printf("Failed to open %s\n", vertexFile.c_str());
            getchar();
            return false;
        }

        printf("Opening fragment shader file..\n");
        // Read the Fragment Shader code from the file
        std::string fragmentShaderCode;
        std::ifstream fragmentShaderStream(fragmentFile.c_str(), std::ios::in);
        if (fragmentShaderStream.is_open())
        {
            std::string Line = "";
            while (getline(fragmentShaderStream, Line))
                fragmentShaderCode += "\n" + Line;
            fragmentShaderStream.close();
        }

        GLint result = GL_FALSE;
        int InfoLogLength;

        // Compile Vertex Shader
        printf("Compiling shader : %s\n", vertexFile.c_str());
        char const * VertexSourcePointer = vertexShaderCode.c_str();
        glShaderSource(vertexShaderID, 1, &VertexSourcePointer, NULL);
        glCompileShader(vertexShaderID);

        // Check Vertex Shader
        glGetShaderiv(vertexShaderID, GL_COMPILE_STATUS, &result);
        glGetShaderiv(vertexShaderID, GL_INFO_LOG_LENGTH, &InfoLogLength);
        if (InfoLogLength > 0)
        {
            std::vector<char> VertexShaderErrorMessage(InfoLogLength + 1);
            glGetShaderInfoLog(vertexShaderID, InfoLogLength, NULL, &VertexShaderErrorMessage[0]);
            printf("%s", &VertexShaderErrorMessage[0]);

        }

        // Compile Fragment Shader
        printf("Compiling shader : %s\n", fragmentFile.c_str());
        char const * FragmentSourcePointer = fragmentShaderCode.c_str();
        glShaderSource(fragmentShaderID, 1, &FragmentSourcePointer, NULL);
        glCompileShader(fragmentShaderID);

        // Check Fragment Shader
        glGetShaderiv(fragmentShaderID, GL_COMPILE_STATUS, &result);
        glGetShaderiv(fragmentShaderID, GL_INFO_LOG_LENGTH, &InfoLogLength);
        if (InfoLogLength > 0)
        {
            std::vector<char> FragmentShaderErrorMessage(InfoLogLength + 1);
            glGetShaderInfoLog(fragmentShaderID, InfoLogLength, NULL, &FragmentShaderErrorMessage[0]);
            printf("%s", &FragmentShaderErrorMessage[0]);
        }

        // Link the program
        printf("Linking program\n");
        programID = glCreateProgram();
        glAttachShader(programID, vertexShaderID);
        glAttachShader(programID, fragmentShaderID);
        glLinkProgram(programID);

        // Check the program
        glGetProgramiv(programID, GL_LINK_STATUS, &result);
        glGetProgramiv(programID, GL_INFO_LOG_LENGTH, &InfoLogLength);
        if (InfoLogLength > 0)
        {
            std::vector<char> ProgramErrorMessage(InfoLogLength + 1);
            glGetProgramInfoLog(programID, InfoLogLength, NULL, &ProgramErrorMessage[0]);
            printf("%s", &ProgramErrorMessage[0]);
        }

        projectionMatrixID = glGetUniformLocation(programID, "Projection");
        viewMatrixID = glGetUniformLocation(programID, "View");
        worldMatrixID = glGetUniformLocation(programID, "World");

        glDeleteShader(vertexShaderID);
        glDeleteShader(fragmentShaderID);
        return true;
    }

}
