#version 330 core
//in vec3 Normal;
//in vec3 FragPos;
out vec4 FragColor;

// uniform vec3 viewPos;

// struct Material {
//     vec3 diffuse;
//     vec3 specular;
//     float shininess;
// };
// uniform Material material;

// // Light properties
// struct Light {
//     vec3 direction;
//     vec3 ambient;
//     vec3 diffuse;
//     vec3 specular;
// };

// uniform Light light;

uniform vec3 uColor;

void main()
{    
    // Ambient reflection color
    // vec3 ambient = light.ambient;
    
    // // Diffuse reflection color
    // vec3 norm = normalize(Normal);
    // vec3 lightDir = normalize(-light.direction); // From fragment to source
    // float diff = max(dot(norm, lightDir), 0.0);
    // vec3 diffuse = light.diffuse * diff;
    
    // // Specular reflection color
    // vec3 viewDir = normalize(viewPos - FragPos);
    // vec3 reflectDir = reflect(-lightDir, norm);
    // float spec = pow(max(dot(viewDir, reflectDir), 0.0), material.shininess);
    // vec3 specular = light.specular * (spec * material.specular);
    
    // Combine the components
    // vec3 result = ambient + diffuse + specular;
    FragColor = vec4(uColor, 1.0);
}