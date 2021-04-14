#version 330 core

in struct fragment_data
{
    vec4 position;
    vec4 normal;
    vec4 color;
    vec4 position_cam_space;
    float floppy_weight;
} fragment;


out vec4 FragColor;

uniform vec3 camera_position;
uniform vec3 color     = vec3(1.0, 1.0, 1.0);
uniform float color_alpha = 1.0;
uniform float ambiant  = 0.2;
uniform float diffuse  = 0.8;
uniform float specular = 0.5;
uniform int specular_exponent = 128;

uniform bool color_weight = false;

vec3 light = vec3(camera_position.x, camera_position.y, camera_position.z);

void main()
{
    vec3 n = normalize(fragment.normal.xyz);
    vec3 u = normalize(light-fragment.position.xyz);
    vec3 r = reflect(u,n);
    vec3 t = normalize(fragment.position.xyz-camera_position);


    float diffuse_value  = diffuse * clamp( dot(u,n), 0.0, 1.0);
    float specular_value = specular * pow( clamp( dot(r,t), 0.0, 1.0), specular_exponent);


    vec3 white = vec3(1.0);
    vec3 c = white;
    if( color_weight==false )
    {
        c = (ambiant+diffuse_value)*color*(1-0.08*abs(fragment.normal.xyz)) + specular_value*white;
    }
    else{
        float a = clamp(fragment.floppy_weight/2.0, 0.0,1.0);
        c = (ambiant+diffuse_value)*vec3(1.0,1.0-a,1.0-a) + specular_value*white;
    }

    float a = clamp(fragment.floppy_weight/2.0, 0.0,1.0);
    vec3 cn = abs(fragment.normal.xyz);
    
    vec3 c1 = vec3(0.95,0.95,1.0);
    vec3 c2 = 0.3*cn*vec3(0.2,0.8,0.8)+vec3(0.9,0.4,0.2);
    vec3 cc = (1-a)*c1 + a * c2;
    c = (ambiant+diffuse_value)*cc /*(vec3(0.95,0.95,1.0)-0.6*a*(0.2*cn*vec3(2,1.5,0.2)))*/ + specular_value*white;

//    float alpha = 1-clamp(-fragment.position_cam_space.z/20.0 , 0.0 , 1.0);
//    alpha = pow(alpha,4.0);
//    c = alpha * c + (1-alpha)*white;

    FragColor = vec4(c, color_alpha);
}
