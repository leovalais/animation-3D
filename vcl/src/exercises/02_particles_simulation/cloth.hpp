#pragma once

#include "../../exercises/base_exercise/base_exercise.hpp"

#ifdef EXERCISE_CLOTH


struct user_parameters_structure
{
    float m;    // Global mass (to be divided by the number of particles)
    float K;    // Spring stiffness
    float mu;   // Damping
    float wind; // Wind magnitude;
};

struct simulation_parameters_structure
{
    float m;  // mass
    float L0; // spring rest length
};

// Sphere and ground used for collision
struct collision_shapes_structure
{
    vcl::vec3 sphere_p;
    float sphere_r;
    float ground_height;
};



struct scene_exercise : base_scene_exercise
{
    // Particles parameters
    std::vector<vcl::vec3> position;
    std::vector<vcl::vec3> speed;
    std::vector<vcl::vec3> force;

    // Simulation parameters
    simulation_parameters_structure simulation_parameters; // parameters that user can control directly
    user_parameters_structure user_parameters;             // parameters adjusted with respect to mesh size (not controled directly by the user)

    // Cloth mesh elements
    vcl::mesh_drawable cloth;              // Visual model for the cloth
    std::vector<vcl::vec3> normals;        // Normal of the cloth used for rendering and wind force computation
    std::vector<vcl::index3> connectivity; // Connectivity of the triangular model

    // Parameters of the shape used for collision
    collision_shapes_structure collision_shapes;

    // Store index and position of vertices constrained to have a fixed 3D position
    std::map<int,vcl::vec3> positional_constraints;

    // Textures
    GLuint texture_cloth;
    GLuint texture_wood;

    // Visual elements of the scene
    vcl::mesh_drawable sphere;
    vcl::mesh_drawable ground;

    // Gui parameters
    bool gui_display_wireframe;
    bool gui_display_texture;

    // Parameters used to control if the simulation runs when a numerical divergence is detected
    bool simulation_diverged; // Active when divergence is detected
    bool force_simulation;    // Force to run simulation even if divergence is detected

    vcl::timer_event timer;



    void initialize();
    void collision_constraints();
    void compute_forces();
    void numerical_integration(float h);
    void detect_simulation_divergence();
    void hard_constraints();
    void set_gui();


    void setup_data(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);
    void frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);
    void display_elements(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);
};






#endif
