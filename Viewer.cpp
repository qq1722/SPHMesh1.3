#include "Viewer.h"
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>
#include <vector>

Viewer::Viewer(int width, int height, const std::string& title)
    : width_(width), height_(height), title_(title), window_(nullptr) {
    init();
}

Viewer::~Viewer() {
    delete shader_;
    delete point_shader_;
    glDeleteVertexArrays(1, &VAO_boundary_); glDeleteBuffers(1, &VBO_boundary_);
    glDeleteVertexArrays(1, &VAO_particles_); glDeleteBuffers(1, &VBO_particles_);
    //glDeleteVertexArrays(1, &VAO_mesh_); glDeleteBuffers(1, &VBO_mesh_); glDeleteBuffers(1, &EBO_mesh_);
    if (window_) { glfwDestroyWindow(window_); }
    glfwTerminate();
}

void Viewer::init() {
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    window_ = glfwCreateWindow(width_, height_, title_.c_str(), nullptr, nullptr);
    glfwMakeContextCurrent(window_);
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) { return; }
    glfwSetWindowUserPointer(window_, this);
    glfwSetFramebufferSizeCallback(window_, framebuffer_size_callback);
    glfwSetMouseButtonCallback(window_, mouse_button_callback);
    glfwSetCursorPosCallback(window_, cursor_pos_callback);
    glfwSetScrollCallback(window_, scroll_callback);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_PROGRAM_POINT_SIZE);
}

void Viewer::set_boundary(Boundary* boundary) {
    boundary_ = boundary;
    if (boundary_) setup_boundary_buffers();
}

void Viewer::set_simulation2d(Simulation2D* sim) {
    sim2d_ = sim;
    if (sim2d_) {
        glGenVertexArrays(1, &VAO_particles_);
        glGenBuffers(1, &VBO_particles_);
        glBindVertexArray(VAO_particles_);
        glBindBuffer(GL_ARRAY_BUFFER, VBO_particles_);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(glm::vec2), (void*)0);
        glEnableVertexAttribArray(0);
        glBindVertexArray(0);
    }
}

void Viewer::set_mesh_generator2d(MeshGenerator2D* generator) {
    generator2d_ = generator;
    if (generator2d_) {
        glGenVertexArrays(1, &VAO_mesh_);
        glGenBuffers(1, &VBO_mesh_);
        glGenBuffers(1, &EBO_mesh_);
        glBindVertexArray(VAO_mesh_);
        glBindBuffer(GL_ARRAY_BUFFER, VBO_mesh_);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(glm::vec2), (void*)0);
        glEnableVertexAttribArray(0);
        glBindVertexArray(0);
    }
}

void Viewer::run() {
    shader_ = new Shader("shaders/simple.vert", "shaders/simple.frag");
    point_shader_ = new Shader("shaders/point.vert", "shaders/point.frag");
    update_camera_vectors();
    main_loop();
}

void Viewer::main_loop() {
    while (!glfwWindowShouldClose(window_)) {
        process_input();

        if (sim2d_) { sim2d_->step(); }
       /* if (generator2d_ && sim2d_ && boundary_) {
            generator2d_->generate(*sim2d_, *boundary_, h_);
        }*/
        update_particle_buffers();
        //update_mesh_buffers();

        glClearColor(0.1f, 0.1f, 0.15f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glm::mat4 model = glm::mat4(1.0f);
        glm::mat4 view = glm::lookAt(camera_pos_, camera_target_, camera_up_);
        glm::mat4 projection = glm::perspective(glm::radians(45.0f), (float)width_ / (float)height_, 0.1f, 100.0f);

        if (boundary_ && shader_) {
            shader_->use();
            shader_->setMat4("model", model);
            shader_->setMat4("view", view);
            shader_->setMat4("projection", projection);
            glLineWidth(1.5f);
            glBindVertexArray(VAO_boundary_);
            glDrawArrays(GL_LINE_LOOP, 0, boundary_->get_vertices().size());
            glLineWidth(1.0f);
        }

      /*  if (generator2d_ && shader_) {
            shader_->use();
            shader_->setMat4("model", model);
            shader_->setMat4("view", view);
            shader_->setMat4("projection", projection);
            glLineWidth(1.5f);
            glBindVertexArray(VAO_mesh_);
            size_t total_line_indices = (generator2d_->get_quads().size() * 4 + generator2d_->get_triangles().size() * 3) * 2;
            if (total_line_indices > 0) {
                glDrawElements(GL_LINES, total_line_indices, GL_UNSIGNED_INT, 0);
            }
            glLineWidth(1.0f);
        }*/

        if (sim2d_ && point_shader_) {
            point_shader_->use();
            point_shader_->setMat4("model", model);
            point_shader_->setMat4("view", view);
            point_shader_->setMat4("projection", projection);
            glBindVertexArray(VAO_particles_);
            glDrawArrays(GL_POINTS, 0, sim2d_->get_particle_positions().size());
        }

        glfwSwapBuffers(window_);
        glfwPollEvents();
    }
}



void Viewer::update_particle_buffers() {
    if (!sim2d_ || sim2d_->get_particle_positions().empty()) return;
    glBindVertexArray(VAO_particles_);
    glBindBuffer(GL_ARRAY_BUFFER, VBO_particles_);
    glBufferData(GL_ARRAY_BUFFER, sim2d_->get_particle_positions().size() * sizeof(glm::vec2), sim2d_->get_particle_positions().data(), GL_DYNAMIC_DRAW);
    glBindVertexArray(0);
}

void Viewer::update_mesh_buffers() {
    if (!generator2d_ || generator2d_->get_vertices().empty()) return;
    glBindVertexArray(VAO_mesh_);
    glBindBuffer(GL_ARRAY_BUFFER, VBO_mesh_);
    glBufferData(GL_ARRAY_BUFFER, generator2d_->get_vertices().size() * sizeof(glm::vec2), generator2d_->get_vertices().data(), GL_DYNAMIC_DRAW);
    const auto& quads = generator2d_->get_quads();
    const auto& triangles = generator2d_->get_triangles();
    std::vector<unsigned int> line_indices;
    line_indices.reserve((quads.size() * 4 + triangles.size() * 3) * 2);
    for (const auto& q : quads) {
        line_indices.push_back(q.v0); line_indices.push_back(q.v1);
        line_indices.push_back(q.v1); line_indices.push_back(q.v2);
        line_indices.push_back(q.v2); line_indices.push_back(q.v3);
        line_indices.push_back(q.v3); line_indices.push_back(q.v0);
    }
    for (const auto& t : triangles) {
        line_indices.push_back(t.v0); line_indices.push_back(t.v1);
        line_indices.push_back(t.v1); line_indices.push_back(t.v2);
        line_indices.push_back(t.v2); line_indices.push_back(t.v0);
    }
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO_mesh_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, line_indices.size() * sizeof(unsigned int), line_indices.data(), GL_DYNAMIC_DRAW);
    glBindVertexArray(0);
}



// --- ìo‘B»ØÕ{º¯”µ ---

void Viewer::setup_boundary_buffers() {
    if (!boundary_) return;
    glGenVertexArrays(1, &VAO_boundary_);
    glGenBuffers(1, &VBO_boundary_);
    glBindVertexArray(VAO_boundary_);
    glBindBuffer(GL_ARRAY_BUFFER, VBO_boundary_);
    glBufferData(GL_ARRAY_BUFFER, boundary_->get_vertices().size() * sizeof(glm::vec2), boundary_->get_vertices().data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(glm::vec2), (void*)0);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);
}

void Viewer::process_input() {
    if (glfwGetKey(window_, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window_, true);
}

void Viewer::update_camera_vectors() {
    float x = camera_target_.x + camera_radius_ * cos(glm::radians(camera_yaw_)) * cos(glm::radians(camera_pitch_));
    float y = camera_target_.y + camera_radius_ * sin(glm::radians(camera_pitch_));
    float z = camera_target_.z + camera_radius_ * sin(glm::radians(camera_yaw_)) * cos(glm::radians(camera_pitch_));
    camera_pos_ = glm::vec3(x, y, z);
}

void Viewer::framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
    auto* viewer = static_cast<Viewer*>(glfwGetWindowUserPointer(window));
    if (viewer) { viewer->width_ = width; viewer->height_ = height; }
}
void Viewer::mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {
    auto* viewer = static_cast<Viewer*>(glfwGetWindowUserPointer(window));
    if (viewer && button == GLFW_MOUSE_BUTTON_LEFT) {
        if (action == GLFW_PRESS) {
            viewer->mouse_left_pressed_ = true;
            glfwGetCursorPos(window, &viewer->last_x_, &viewer->last_y_);
        }
        else if (action == GLFW_RELEASE) {
            viewer->mouse_left_pressed_ = false;
        }
    }
}
void Viewer::cursor_pos_callback(GLFWwindow* window, double xpos, double ypos) {
    auto* viewer = static_cast<Viewer*>(glfwGetWindowUserPointer(window));
    if (viewer && viewer->mouse_left_pressed_) {
        float xoffset = xpos - viewer->last_x_;
        float yoffset = viewer->last_y_ - ypos;
        viewer->last_x_ = xpos;
        viewer->last_y_ = ypos;
        float sensitivity = 0.2f;
        viewer->camera_yaw_ += xoffset * sensitivity;
        viewer->camera_pitch_ += yoffset * sensitivity;
        if (viewer->camera_pitch_ > 89.0f) viewer->camera_pitch_ = 89.0f;
        if (viewer->camera_pitch_ < -89.0f) viewer->camera_pitch_ = -89.0f;
        viewer->update_camera_vectors();
    }
}
void Viewer::scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
    auto* viewer = static_cast<Viewer*>(glfwGetWindowUserPointer(window));
    if (viewer) {
        viewer->camera_radius_ -= (float)yoffset * 0.5f;
        if (viewer->camera_radius_ < 1.0f) viewer->camera_radius_ = 1.0f;
        if (viewer->camera_radius_ > 40.0f) viewer->camera_radius_ = 40.0f;
        viewer->update_camera_vectors();
    }
}