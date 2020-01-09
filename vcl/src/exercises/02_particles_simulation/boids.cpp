
#include "boids.hpp"

#include <random>
#include <algorithm>
#include <chrono>
#include <optional>

#ifdef EXERCISE_BOIDS

using namespace vcl;



void add_boids(std::vector<particle_structure>& particles, std::vector<vcl::curve_dynamic_drawable>& trajectories);
static void set_gui(std::vector<particle_structure>& particles,timer_basic& timer, std::vector<vcl::curve_dynamic_drawable>& trajectories);

static unsigned long int get_elapsed_time() {
    static auto start = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
    start = now;
    return elapsed;
}

static std::optional<unsigned long int>
hacked_timer(std::optional<unsigned long int> time_ms = std::nullopt) {
    static unsigned long int time_left = 0;
    if (time_ms == std::nullopt) {
        auto elapsed = get_elapsed_time();
        time_left = elapsed > time_left ? 0 : time_left - elapsed;
        if (time_left == 0)
            return std::nullopt;
        else
            return time_left;
    }
    else {
        time_left = *time_ms;
        return time_left;
    }
}

void scene_exercise::setup_data(std::map<std::string,GLuint>& , scene_structure&, gui_structure&)
{
    // Create boids
    add_boids(particles, trajectories);

    // Initialize the visual model of boids
    cone = mesh_drawable(mesh_primitive_cone(0.1f, {0,0,0}, {0,0,0.5f}));
    cone.uniform_parameter.scaling = 0.15f;
    cone.uniform_parameter.color = {0,0,1};

    // init time counter
    get_elapsed_time();
}

template <typename T>
static inline T constrain(const T& min, const T& val, const T& max) {
    return std::max(std::min(val, max), min);
}

static inline vec3 constrain(const vec3& min, const vec3& val, const vec3& max) {
    return {constrain(min.x, val.x, max.x),
            constrain(min.y, val.y, max.y),
            constrain(min.z, val.z, max.z)};
}


/**
 * Positive return value => get further
 * Negative return value => get closer
 */
float force(float d)
{
    if (d < 0.0000000001f) // == 0
        return 0.f;
    constexpr float k = 0.5f;
    float dk = k - d; // dk < 0 => get closer, dk > 0 => get further
    return dk < 0 ? -1.f / (dk * dk) : 1.f / dk;
}

static inline vec3 normalize(const vec3& v) {
    return v / norm(v);
}

void scene_exercise::frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& )
{
    constexpr float max_force = 0.4f;
    constexpr float max_direction = 0.1f;

    const float dt = timer.update();
    set_gui(particles,timer, trajectories);

    // Initialize forces to zero
    const size_t N = particles.size();
    for(size_t k=0; k<N; ++k)
        particles[k].f = {0,0,0};

    // static int dir_idx = 0;
    // static const vec3 directions[] = {{0,1,0}, {1,0,0}, {0.5,0.5,0}, {0.5,-0.5,0}, {-0.5,0.5,0}, {-0.5,0.5,0}, {0,-1,0}, {-1,0,0}};
    // constexpr auto dir_count = 8;
    static vec3 direction = {0.0,0.0,0.0};

    if (!hacked_timer()) {
        hacked_timer(2000);
        // direction = directions[dir_idx++ % dir_count];
        const auto between_minus_one_and_one = [](){
            return 2.f * (static_cast<float>(rand()) / static_cast<float>(RAND_MAX)) - 1.f;
        };
        direction = normalize({between_minus_one_and_one(), between_minus_one_and_one(), 0.f});
    }

    // 
    // Add forces ...
    for(int i=0; i<N; ++i)
    {
        const vec3& p = particles[i].p;
        const float f = expf(norm(p) / 2.f);
        particles[i].f += f * normalize(-normalize(p));

        for(int j=i+1; j<N; ++j)
        {
                const vec3& pi = particles[i].p;
                const vec3& pj = particles[j].p;
                float f = force(norm(pi - pj));
                f = constrain(-max_force, f, max_force);
                vec3 fv = f * normalize(pi - pj);
                particles[i].f += fv;
                particles[j].f += -fv;
                particles[i].f += max_direction * direction;
                particles[j].f += max_direction * direction;
        }
    }

    // 
    // Integrate position and speed of boids through time
    for(size_t k=0; k<N; ++k)
    {
        particle_structure& particle = particles[k];

        const vec3 max_speed = {5, 5, 0};
        particle.v = constrain(-max_speed, particle.v + dt*particle.f, max_speed);
        particle.p = particle.p + dt*particle.v;
        trajectories[k].add_point(particle.p);
    }

    // 
    // Display of boids
    for(size_t k=0; k<N; ++k)
    {
        const vec3& p = particles[k].p;
        const vec3& v = particles[k].v;
        const mat3 R = rotation_between_vector_mat3({0,0,1},v);

        cone.uniform_parameter.translation = p;
        cone.uniform_parameter.rotation = R;

        cone.draw(shaders["mesh"], scene.camera);
    }

    //a commenter pour virer les traces
    // for(size_t k=0; k<N; ++k)
    // {
    //     trajectories[k].draw(shaders["curve"], scene.camera);
    // }
}


/** Add new boids in the scene */
void add_boids(std::vector<particle_structure>& particles, std::vector<vcl::curve_dynamic_drawable>& trajectories)
{
    // Max number of boids to be added
    const size_t N = 50;


    for(size_t k=0; k<N; ++k)
    {
        // Initial position and speed distribution is 2D to simplify the visualization

        particle_structure particle;
        particle.p = {rand_interval(-0.5f,0.5f), rand_interval(-0.5f,0.5f), 0};
        particle.p *= 2.0f;

        const float theta = 2*float(M_PI)*rand_interval();
        particle.v = {std::cos(theta),std::sin(theta),0};
        particle.f = {0,0,0};

        particles.push_back(particle);
    }

    for (size_t i = 0; i < N; i++)
    {
        vcl::curve_dynamic_drawable trajectory = curve_dynamic_drawable(100);
        trajectory.uniform_parameter.color = {0,0,1};
        trajectories.push_back(trajectory);
    }
}


static void set_gui(std::vector<particle_structure>& particles,timer_basic& timer, std::vector<vcl::curve_dynamic_drawable>& trajectories)
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

    if (ImGui::Button("Add boids"))
        add_boids(particles, trajectories);
}



#endif
