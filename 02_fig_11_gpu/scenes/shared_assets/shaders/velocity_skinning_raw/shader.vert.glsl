#version 330 core

layout (location = 0) in vec4 position;
layout (location = 1) in vec4 normal;


out struct fragment_data
{
    vec4 position;
    vec4 normal;
    vec4 color;
    vec4 position_cam_space;
    float floppy_weight;
} fragment;


// model transformation
uniform vec3 translation = vec3(0.0, 0.0, 0.0);                      // user defined translation
uniform mat3 rotation = mat3(1.0,0.0,0.0, 0.0,1.0,0.0, 0.0,0.0,1.0); // user defined rotation
uniform float scaling = 1.0;                                         // user defined scaling
uniform vec3 scaling_axis = vec3(1.0,1.0,1.0);                       // user defined scaling
uniform int N_vertex;
uniform int N_joint;
uniform float floppy_power=1.0;

// view transform
uniform mat4 view;
// perspective matrix
uniform mat4 perspective;


uniform samplerBuffer tbo_weights;
uniform samplerBuffer tbo_velocity_weights;

uniform samplerBuffer tbo_sk0;
uniform samplerBuffer tbo_sk;
uniform samplerBuffer tbo_angular_velocity;
uniform samplerBuffer texture_floppy_weights_raw;

uniform isamplerBuffer texture_rig_joint;
uniform samplerBuffer texture_rig_weight;
uniform isamplerBuffer texture_rig_cumulative_index;
uniform isamplerBuffer texture_rig_size;

uniform isamplerBuffer texture_vs_rig_joint;
uniform samplerBuffer texture_vs_rig_weight;
uniform isamplerBuffer texture_vs_rig_cumulative_index;
uniform isamplerBuffer texture_vs_rig_size;

uniform bool floppy_weight_active = false;
uniform bool floppy_limit = false;
uniform bool gpu_velocity_skinning = true;


mat3 rotation_from_axis_angle(vec3 u, float angle)
{
    float x = u.x;
    float y = u.y;
    float z = u.z;
    float c = cos(angle);
    float s = sin(angle);

    mat3 R = mat3(vec3(c+x*x*(1-c)  , y*x*(1-c)+z*s, z*x*(1-c)-y*s), vec3(x*y*(1-c)-z*s, c+y*y*(1-c), z*y*(1-c)+x*s), vec3(x*z*(1-c)+y*s, y*z*(1-c)-x*s, c+z*z*(1-c)) );
    return R;
}

mat4 mat4_from_texture(samplerBuffer s, int k)
{
        float xx = texelFetch(s, 16*k+0).r; float xy = texelFetch(s, 16*k+1).r; float xz = texelFetch(s, 16*k+2).r; float xw = texelFetch(s, 16*k+3).r;
        float yx = texelFetch(s, 16*k+4).r; float yy = texelFetch(s, 16*k+5).r; float yz = texelFetch(s, 16*k+6).r; float yw = texelFetch(s, 16*k+7).r;
        float zx = texelFetch(s, 16*k+8).r; float zy = texelFetch(s, 16*k+9).r; float zz = texelFetch(s, 16*k+10).r; float zw = texelFetch(s, 16*k+11).r;
        float wx = texelFetch(s, 16*k+12).r; float wy = texelFetch(s, 16*k+13).r; float wz = texelFetch(s, 16*k+14).r; float ww = texelFetch(s, 16*k+15).r;

        return mat4(vec4(xx,yx,zx,wx), vec4(xy,yy,zy,wy), vec4(xz,yz,zz,wz), vec4(xw,yw,zw,ww));
}
vec3 translation_from_mat4_texture(samplerBuffer s, int k)
{
        float xx = texelFetch(s, 16*k+0).r; float xy = texelFetch(s, 16*k+1).r; float xz = texelFetch(s, 16*k+2).r; float xw = texelFetch(s, 16*k+3).r;
        float yx = texelFetch(s, 16*k+4).r; float yy = texelFetch(s, 16*k+5).r; float yz = texelFetch(s, 16*k+6).r; float yw = texelFetch(s, 16*k+7).r;
        float zx = texelFetch(s, 16*k+8).r; float zy = texelFetch(s, 16*k+9).r; float zz = texelFetch(s, 16*k+10).r; float zw = texelFetch(s, 16*k+11).r;
        float wx = texelFetch(s, 16*k+12).r; float wy = texelFetch(s, 16*k+13).r; float wz = texelFetch(s, 16*k+14).r; float ww = texelFetch(s, 16*k+15).r;

        return vec3(texelFetch(s, 16*k+3).r, texelFetch(s, 16*k+7).r, texelFetch(s, 16*k+11).r);
}


void main()
{
    // scaling matrix
    mat4 Sm = mat4(scaling*scaling_axis.x,0.0,0.0,0.0, 0.0,scaling*scaling_axis.y,0.0,0.0, 0.0,0.0,scaling*scaling_axis.z,0.0, 0.0,0.0,0.0,1.0);
    // 4x4 rotation matrix
    mat4 Rm = mat4(rotation);
    // 4D translation
    vec4 Tm = vec4(translation,0.0);
    //Tm.x += 2*gl_InstanceID;

    vec4 position_transformed = position;
    vec4 normal_transformed = normal;

    //vec3 p_skinning3d = position.xyz;
    //vec4 p_skinning_normal = normal;

    vec4 p_skinning = vec4(0.0, 0.0, 0.0, 0.0);
    vec4 p_skinning_normal = vec4(0.0, 0.0, 0.0, 0.0);

    int offset = texelFetch(texture_rig_cumulative_index, gl_VertexID).r;
    int N_dependency = texelFetch(texture_rig_size, gl_VertexID).r;
    for(int k_dep=0; k_dep<N_dependency; ++k_dep)
    {
        int joint = texelFetch(texture_rig_joint, offset+k_dep).r;
        float w = texelFetch(texture_rig_weight, offset+k_dep).r;

        mat4 T = mat4_from_texture(tbo_sk, joint);
        mat4 T0 = mat4_from_texture(tbo_sk0, joint);
        
        p_skinning = p_skinning + w * T * T0 * position_transformed;
        p_skinning_normal = p_skinning_normal + w * T * T0 * vec4(normal_transformed.xyz,0.0);
    }

//    for(int k_joint=0; k_joint<N_joint; k_joint++)
//    {
//        mat4 T = mat4_from_texture(tbo_sk, k_joint);
//        mat4 T0 = mat4_from_texture(tbo_sk0, k_joint);
//        
//        float w = texelFetch(tbo_weights, gl_VertexID+k_joint*N_vertex).r;
//
//        p_skinning = p_skinning + w * T * T0 * position_transformed;
//        p_skinning_normal = p_skinning_normal + w * T * T0 * vec4(normal_transformed.xyz,0.0);
//    }

    vec3 p_skinning3d = p_skinning.xyz;
    

    p_skinning_normal = normalize(p_skinning_normal);
    vec3 p_skinning_normal3d = p_skinning_normal.xyz;

    float floppy_weight_vertex = 1.0f;
    
    fragment.floppy_weight = texelFetch(texture_floppy_weights_raw,gl_VertexID).r;
    if(gpu_velocity_skinning==true)
    {
    
        if(floppy_weight_active)
        {
            floppy_weight_vertex = texelFetch(texture_floppy_weights_raw,gl_VertexID).r;
        }
        
        fragment.floppy_weight = floppy_weight_vertex;
        
        // HACK - floppy
        //floppy_weight_vertex = 1.0f;

        mat3 Id = mat3(vec3(1,0,0),vec3(0,1,0),vec3(0,0,1));

        int vs_offset = texelFetch(texture_vs_rig_cumulative_index, gl_VertexID).r;
        int vs_N_dependency = texelFetch(texture_vs_rig_size, gl_VertexID).r;

        for(int k_dep=0; k_dep<vs_N_dependency; ++k_dep)
        {
            int k_joint = texelFetch(texture_vs_rig_joint, vs_offset+k_dep).r;
            float w = texelFetch(texture_vs_rig_weight, vs_offset+k_dep).r;

            vec3 omega = vec3(texelFetch(tbo_angular_velocity, 3*k_joint+0).r,texelFetch(tbo_angular_velocity, 3*k_joint+1).r,texelFetch(tbo_angular_velocity, 3*k_joint+2).r);//+0.01*angular_velocity[k_joint];
            if( length(omega)>0.001 ){
            
                vec3 un_angular_velocity = normalize(omega);
                //float w = texelFetch(tbo_velocity_weights, gl_VertexID+k_joint*N_vertex).r;
                vec3 p_joint = translation_from_mat4_texture(tbo_sk, k_joint);//vec3(skeleton[k_joint][0][3],skeleton[k_joint][1][3],skeleton[k_joint][2][3]);
                vec3 u_joint = p_skinning3d - p_joint;
                vec3 rotation_center = p_joint + dot(u_joint, un_angular_velocity)*un_angular_velocity;
                float vertex_speed_norm = length(omega) * length(u_joint);
                float rotation_angle = floppy_weight_vertex * floppy_power * 0.2 * vertex_speed_norm;
            
                float m = 3.14/12.0;
                if(rotation_angle>m && floppy_limit==true)
                    rotation_angle = m + (rotation_angle-m)/vertex_speed_norm;

                mat3 R = rotation_from_axis_angle(un_angular_velocity, -rotation_angle);

                p_skinning3d = p_skinning3d + w * (R-Id)*(p_skinning3d-rotation_center);
                p_skinning_normal3d = p_skinning_normal3d + w * (R-Id)*(p_skinning3d-rotation_center);
            }
        }
    }
    


//    for(int k_joint=0; k_joint<N_joint; k_joint++)
//    {
//
//        vec3 omega = vec3(texelFetch(tbo_angular_velocity, 3*k_joint+0).r,texelFetch(tbo_angular_velocity, 3*k_joint+1).r,texelFetch(tbo_angular_velocity, 3*k_joint+2).r);//+0.01*angular_velocity[k_joint];
//        if( length(omega)>0.001 ){
//            
//            vec3 un_angular_velocity = normalize(omega);
//            float w = texelFetch(tbo_velocity_weights, gl_VertexID+k_joint*N_vertex).r;
//            vec3 p_joint = translation_from_mat4_texture(tbo_sk, k_joint);//vec3(skeleton[k_joint][0][3],skeleton[k_joint][1][3],skeleton[k_joint][2][3]);
//            vec3 u_joint = p_skinning3d - p_joint;
//            vec3 rotation_center = p_joint + dot(u_joint, un_angular_velocity)*un_angular_velocity;
//            float vertex_speed_norm = length(omega) * length(u_joint);
//            float rotation_angle = floppy_weight_vertex * floppy_power * 0.2 * vertex_speed_norm;
//            
//            float m = 3.14/12.0;
//            if(rotation_angle>m)
//                rotation_angle = m + (rotation_angle-m)/vertex_speed_norm;
//
//            mat3 R = rotation_from_axis_angle(un_angular_velocity, -rotation_angle);
//
//            p_skinning3d = p_skinning3d + w * (R-Id)*(p_skinning3d-rotation_center);
//            p_skinning_normal3d = p_skinning_normal3d + w * (R-Id)*(p_skinning3d-rotation_center);
//        }
//    }




    //p_skinning3d = p_skinning3d;
    p_skinning_normal3d = normalize(p_skinning_normal3d);
    p_skinning_normal = vec4(p_skinning_normal3d,0.0);

    position_transformed = Rm*Sm*vec4(p_skinning3d,1.0)+Tm;
    
    fragment.normal = Rm*p_skinning_normal;
    fragment.position = position_transformed;
    fragment.color = vec4(floppy_weight_vertex,0,0,0);
    fragment.position_cam_space = view * position_transformed;
    gl_Position = perspective * fragment.position_cam_space;
}
