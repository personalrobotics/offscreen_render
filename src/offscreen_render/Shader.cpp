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
        if (programID)
            glDeleteProgram(programID);
    }

    bool Shader::LoadFromFile(const std::string& fragmentFile, const std::string& vertexFile)
    {
        // Create the shaders
        ProgramID vertexShaderID = glCreateShader(GL_VERTEX_SHADER);
        checkError("glCreateShader");
        ProgramID fragmentShaderID = glCreateShader(GL_FRAGMENT_SHADER);
        checkError("glCreateShader");

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
        char const * VertexSourcePointer = vertexShaderCode.c_str();
        glShaderSource(vertexShaderID, 1, &VertexSourcePointer, NULL);
        glCompileShader(vertexShaderID);
        checkError("glCompileShader");

        // Check Vertex Shader
        glGetShaderiv(vertexShaderID, GL_COMPILE_STATUS, &result);
        glGetShaderiv(vertexShaderID, GL_INFO_LOG_LENGTH, &InfoLogLength);
        if (InfoLogLength > 0)
        {
            std::vector<char> VertexShaderErrorMessage(InfoLogLength + 1);
            glGetShaderInfoLog(vertexShaderID, InfoLogLength, NULL, &VertexShaderErrorMessage[0]);
            fprintf(stderr, "%s", &VertexShaderErrorMessage[0]);
        }

        // Compile Fragment Shader
        char const * FragmentSourcePointer = fragmentShaderCode.c_str();
        glShaderSource(fragmentShaderID, 1, &FragmentSourcePointer, NULL);
        glCompileShader(fragmentShaderID);
        checkError("glCompileShader");

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
        programID = glCreateProgram();
        glAttachShader(programID, vertexShaderID);
        checkError("glAttachShader");
        glAttachShader(programID, fragmentShaderID);
        checkError("glAttachShader");

        glBindAttribLocation(programID, 0, "position");
        checkError("glBindAttribLocation");
        glBindAttribLocation(programID, 1, "color");
        checkError("glBindAttribLocation");

        glLinkProgram(programID);
        checkError("glLinkProgram");

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
        checkError("glGetUniformLocation");
        viewMatrixID = glGetUniformLocation(programID, "View");
        checkError("glGetUniformLocation");
        worldMatrixID = glGetUniformLocation(programID, "World");
        checkError("glGetUniformLocation");

        glDeleteShader(vertexShaderID);
        glDeleteShader(fragmentShaderID);
        return true;
    }

}
