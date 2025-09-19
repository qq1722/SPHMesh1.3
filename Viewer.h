#pragma once
#include <glad/glad.h>
#include <glfw/glfw3.h>
#include <glm/glm.hpp>
#include <string>

#include "Shader.h"
#include "Boundary.h"
#include "Simulation2D.h"
#include "MeshGenerator2D.h"

class Viewer {
public:
    Viewer(int width, int height, const std::string& title);
    ~Viewer();

    Viewer(const Viewer&) = delete;
    Viewer& operator=(const Viewer&) = delete;

    void set_boundary(Boundary* boundary);
    void set_simulation2d(Simulation2D* sim);
    void set_mesh_generator2d(MeshGenerator2D* generator);
    void run();

private:
    void init();
    void main_loop();
    void process_input();
    void update_camera_vectors();

    void setup_boundary_buffers();
    void update_particle_buffers();
    void update_mesh_buffers();

    static void framebuffer_size_callback(GLFWwindow* window, int width, int height);
    static void mouse_button_callback(GLFWwindow* window, int button, int action, int mods);
    static void cursor_pos_callback(GLFWwindow* window, double xpos, double ypos);
    static void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);

private:
    GLFWwindow* window_;
    int width_, height_;
    std::string title_;

    Shader* shader_ = nullptr;
    Shader* point_shader_ = nullptr;
    unsigned int VAO_boundary_ = 0, VBO_boundary_ = 0;
    unsigned int VAO_particles_ = 0, VBO_particles_ = 0;
    unsigned int VAO_mesh_ = 0, VBO_mesh_ = 0, EBO_mesh_ = 0;

    Boundary* boundary_ = nullptr;
    Simulation2D* sim2d_ = nullptr;
    MeshGenerator2D* generator2d_ = nullptr;

    glm::vec3 camera_target_ = glm::vec3(0.0f, 0.0f, 0.0f);
    glm::vec3 camera_up_ = glm::vec3(0.0f, 1.0f, 0.0f);
    glm::vec3 camera_pos_;
    float camera_radius_ = 15.0f;
    float camera_yaw_ = -90.0f;
    float camera_pitch_ = 0.0f;

    bool mouse_left_pressed_ = false;
    double last_x_, last_y_;

    float h_ = 0.10f;
};