#pragma once
#include <glad/glad.h>
#include <glfw/glfw3.h>
#include <glm/glm.hpp>
#include <string>
#include <fstream>

#include "Shader.h"
#include "Boundary.h"
#include "Simulation2D.h"
#include "MeshGenerator2D.h"
#include "BackgroundGrid.h" 

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
    void set_background_grid(BackgroundGrid* grid); // 新增

private:
    void init();
    void main_loop();
    //void process_input();
    // 新增：键盘回调函数
    static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
    void update_camera_vectors();

    void setup_boundary_buffers();
    void update_particle_buffers();
   // void update_mesh_buffers();

    static void framebuffer_size_callback(GLFWwindow* window, int width, int height);
    static void mouse_button_callback(GLFWwindow* window, int button, int action, int mods);
    static void cursor_pos_callback(GLFWwindow* window, double xpos, double ypos);
    static void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);

    // 新增：可视化模式切换和数据导出
    void toggle_view_mode();
    void save_particle_snapshot();
    // 新增：为大小场设置缓冲区
    void setup_size_field_buffers();

private:
    GLFWwindow* window_;
    int width_, height_;
    std::string title_;

    Shader* shader_ = nullptr;
    Shader* point_shader_ = nullptr;

    Shader* size_field_shader_ = nullptr;
    unsigned int VAO_size_field_ = 0, VBO_size_field_ = 0;
    BackgroundGrid* grid_ = nullptr;
    // 控制逻辑
    bool show_size_field_ = false;
    int step_count_ = 0;
    std::ofstream convergence_log_;


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