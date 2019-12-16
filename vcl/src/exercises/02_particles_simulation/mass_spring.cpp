
#include "mass_spring.hpp"


#ifdef EXERCISE_MASS_SPRING

using namespace vcl;


static void set_gui(timer_basic& timer);


/** Compute spring force applied on pi with respect to pj */
vec3 spring_force(const vec3& pi, const vec3& pj, float L0, float K)
{
    const vec3 p = pi-pj;
    float L = norm(p);
    const vec3 u = normalize(p);

    const vec3 F = -K * (L-L0) * u;
    return F;
}


void scene_exercise::setup_data(std::map<std::string,GLuint>& shaders, scene_structure& , gui_structure& gui)
{
    gui.show_frame_camera = false;
    shaders["segment_immediate_mode"] = create_shader_program("shaders/segment_immediate_mode/shader.vert.glsl","shaders/segment_immediate_mode/shader.frag.glsl");
    segment_drawer.init();
    sphere = mesh_drawable( mesh_primitive_sphere(1.0f));


    std::vector<vec3> borders_segments = {{-1,-1,-1},{1,-1,-1}, {1,-1,-1},{1,1,-1}, {1,1,-1},{-1,1,-1}, {-1,1,-1},{-1,-1,-1},
                                          {-1,-1,1} ,{1,-1,1},  {1,-1,1}, {1,1,1},  {1,1,1}, {-1,1,1},  {-1,1,1}, {-1,-1,1},
                                          {-1,-1,-1},{-1,-1,1}, {1,-1,-1},{1,-1,1}, {1,1,-1},{1,1,1},   {-1,1,-1},{-1,1,1}};
    borders = segments_gpu(borders_segments);
    borders.uniform_parameter.color = {0,0,0};


    pA.p = {0,0,0};
    pA.v = {0,0,0};

    pB.p = {0.5f,0,0};
    pB.v = {0,0,0};

    L0 = 0.4f; // Rest length between A and B

}





void scene_exercise::frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& )
{
    const float dt = timer.update();
    set_gui(timer);

    // Force constant time step
    float h = dt<=1e-6f? 0.0f : timer.scale*0.01f;


    // Simulation parameters
    const float m  = 0.05f;        // particle mass
    const float K  = 5.0f;         // spring stiffness

    const vec3 g   = {0,-9.81f,0}; // gravity

    // Forces
    const vec3 f_spring  = spring_force(pB.p, pA.p, L0, K);
    const vec3 f_weight  = m*g;

    const vec3 F = f_spring+f_weight;

    // Numerical Integration (Explicit Euler: change it to Verlet)
    {
        // Only particle B should be updated
        vec3& p = pB.p; // position of particle
        vec3& v = pB.v; // speed of particle

        p = p + h*v;
        v = v + h*F/m;
    }




    // Display

    // particle pa
    sphere.uniform_parameter.translation = pA.p;
    sphere.uniform_parameter.scaling = 0.05f;
    sphere.uniform_parameter.color = {0,0,0};
    sphere.draw(shaders["mesh"], scene.camera);

    // particle pb
    sphere.uniform_parameter.translation = pB.p;
    sphere.uniform_parameter.scaling = 0.05f;
    sphere.uniform_parameter.color = {1,0,0};
    sphere.draw(shaders["mesh"], scene.camera);

    // Spring pa-pb
    segment_drawer.uniform_parameter.p1 = pA.p;
    segment_drawer.uniform_parameter.p2 = pB.p;
    segment_drawer.draw(shaders["segment_immediate_mode"],scene.camera);



    borders.draw(shaders["curve"], scene.camera);
}


/** Part specific GUI drawing */
static void set_gui(timer_basic& timer)
{
    // Can set the speed of the animation
    float scale_min = 0.05f;
    float scale_max = 2.0f;
    ImGui::SliderScalar("Time scale", ImGuiDataType_Float, &timer.scale, &scale_min, &scale_max, "%.2f s");

    // Start and stop animation
    if (ImGui::Button("Stop"))
        timer.stop();
    if (ImGui::Button("Start"))
        timer.start();

}



#endif
