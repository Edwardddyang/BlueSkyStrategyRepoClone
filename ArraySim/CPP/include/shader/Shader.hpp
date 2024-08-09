#ifndef SHADER_H
#define SHADER_H

/* Manages a vertex + fragment shader combination, and provides function to manage
    uniform properties */

#include <string>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

class Shader
{
private:
    unsigned int uid; /* id of the shader program */
public:
    unsigned int get_uid() const {return uid;};
  
    // constructor reads and builds the shader
    Shader(const char* vertexPath, const char* fragmentPath);
    Shader() {}
    // use/activate the shader
    void use();
    // utility uniform functions
    void setBool(const std::string &name, bool value) const;  
    void setInt(const std::string &name, int value) const;   
    void setFloat(const std::string &name, float value) const;
    void setVec3(const std::string &name, const glm::vec3 &value) const;
    void setVec3(const std::string &name, float x, float y, float z) const;
    void setMat4(const std::string &name, const glm::mat4 &mat) const;
};
  
#endif