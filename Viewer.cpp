#include "Viewer.h"
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>
#include <vector>
#include <algorithm>

// --- 构造函数：打开日志文件 ---
Viewer::Viewer(int width, int height, const std::string& title)
    : width_(width), height_(height), title_(title), window_(nullptr) {
    init();
    convergence_log_.open("convergence_log.csv");
    if (convergence_log_.is_open()) {
        convergence_log_ << "Step,KineticEnergy\n"; // 写入CSV表头
    }
}

// --- 析构函数：关闭日志文件 ---
Viewer::~Viewer() {
    delete shader_;
    delete point_shader_;
    delete size_field_shader_;
    glDeleteVertexArrays(1, &VAO_boundary_); glDeleteBuffers(1, &VBO_boundary_);
    glDeleteVertexArrays(1, &VAO_particles_); glDeleteBuffers(1, &VBO_particles_);
    glDeleteVertexArrays(1, &VAO_size_field_); glDeleteBuffers(1, &VBO_size_field_);
    if (window_) { glfwDestroyWindow(window_); }
    if (convergence_log_.is_open()) {
        convergence_log_.close();
    }
    glfwTerminate();
}

void Viewer::set_background_grid(BackgroundGrid* grid) {
    grid_ = grid;
    if (grid_) {
        setup_size_field_buffers();
    }
}






// --- 主循环：添加新逻辑 ---
void Viewer::main_loop() {
    while (!glfwWindowShouldClose(window_)) {
        if (glfwGetKey(window_, GLFW_KEY_ESCAPE) == GLFW_PRESS)
            glfwSetWindowShouldClose(window_, true);

        if (sim2d_) {
            sim2d_->step();
            step_count_++;
            if (step_count_ % 10 == 0 && convergence_log_.is_open()) {
                convergence_log_ << step_count_ << "," << sim2d_->get_kinetic_energy() << "\n";
            }
        }

        update_particle_buffers();

        glClearColor(0.1f, 0.1f, 0.15f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glm::mat4 model = glm::mat4(1.0f);
        glm::mat4 view = glm::lookAt(camera_pos_, camera_target_, camera_up_);
        glm::mat4 projection = glm::perspective(glm::radians(45.0f), (float)width_ / (float)height_, 0.1f, 100.0f);

        if (show_mesh_ && delaunay_generator_ && shader_) {
            shader_->use();
            shader_->setMat4("view", view);
            shader_->setMat4("projection", projection);

            // 绘制网格线
            shader_->setVec4("color", glm::vec4(0.9f, 0.9f, 0.9f, 1.0f)); // 浅灰色
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            glLineWidth(1.0f);
            glBindVertexArray(VAO_mesh_);
            glDrawElements(GL_TRIANGLES, delaunay_generator_->get_triangles().size() * 3, GL_UNSIGNED_INT, 0);
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        }
        
        else if (show_size_field_) {
            // --- 绘制大小场 ---
            if (grid_ && size_field_shader_) {
                size_field_shader_->use();
                size_field_shader_->setMat4("view", view);
                size_field_shader_->setMat4("projection", projection);
                const auto& field = grid_->get_target_size_field();
                if (!field.empty()) {
                    float min_val = *std::min_element(field.begin(), field.end());
                    float max_val = *std::max_element(field.begin(), field.end());
                    size_field_shader_->setFloat("vmin", min_val);
                    size_field_shader_->setFloat("vmax", max_val);
                }
                glBindVertexArray(VAO_size_field_);
                glDrawArrays(GL_TRIANGLES, 0, grid_->get_width() * grid_->get_height() * 6);
            }
            // --- 在大小场之上叠加边界线 ---
            if (boundary_ && shader_) {
                shader_->use();
                shader_->setMat4("view", view);
                shader_->setMat4("projection", projection);
                glLineWidth(1.0f);
                glBindVertexArray(VAO_boundary_);
                glDrawArrays(GL_LINE_LOOP, 0, boundary_->get_vertices().size());
            }
        }
        
        else {
            // 绘制边界
            if (boundary_ && shader_) {
                shader_->use();
                shader_->setMat4("model", model);
                shader_->setMat4("view", view);
                shader_->setMat4("projection", projection);
                glLineWidth(1.0f);
                glBindVertexArray(VAO_boundary_);
                glDrawArrays(GL_LINE_LOOP, 0, boundary_->get_vertices().size());
                glLineWidth(1.0f);
            }
            // 绘制粒子
            if (sim2d_ && point_shader_) {
                point_shader_->use();
                shader_->setMat4("model", model);
                point_shader_->setMat4("view", view);
                point_shader_->setMat4("projection", projection);
                glPointSize(1.0f);
                glBindVertexArray(VAO_particles_);
                if (!sim2d_->get_particle_positions().empty()) {
                    glDrawArrays(GL_POINTS, 0, sim2d_->get_particle_positions().size());
                }
            }
        }

        glfwSwapBuffers(window_);
        glfwPollEvents();
    }
}



// --- 新增函数 ---

void Viewer::update_mesh_buffers() {
    if (!delaunay_generator_ || delaunay_generator_->get_vertices().empty()) return;

    glBindVertexArray(VAO_mesh_);

    glBindBuffer(GL_ARRAY_BUFFER, VBO_mesh_);
    glBufferData(GL_ARRAY_BUFFER, delaunay_generator_->get_vertices().size() * sizeof(glm::vec2), delaunay_generator_->get_vertices().data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO_mesh_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, delaunay_generator_->get_triangles().size() * sizeof(DelaunayMeshGenerator::Triangle), delaunay_generator_->get_triangles().data(), GL_STATIC_DRAW);

    glBindVertexArray(0);
}


void Viewer::set_delaunay_generator(DelaunayMeshGenerator* generator) {
    delaunay_generator_ = generator;
    if (delaunay_generator_) {
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

void Viewer::toggle_view_mode() {
    show_size_field_ = !show_size_field_;
    if (show_size_field_) {
        std::cout << "Displaying: Size Field (h_t). Press V to switch back to particles." << std::endl;
    }
    else {
        std::cout << "Displaying: Particles. Press V to switch to Size Field." << std::endl;
    }
}

void Viewer::save_particle_snapshot() {
    if (!sim2d_) return;
    std::string filename = "particles_step_" + std::to_string(step_count_) + ".txt";
    std::ofstream outfile(filename);
    if (outfile.is_open()) {
        outfile << "x,y\n"; // CSV header
        for (const auto& pos : sim2d_->get_particle_positions()) {
            outfile << pos.x << "," << pos.y << "\n";
        }
        outfile.close();
        std::cout << "Saved particle snapshot to " << filename << std::endl;
    }
}

void Viewer::setup_size_field_buffers() {
    if (!grid_) return;
    size_field_shader_ = new Shader("shaders/size_field.vert", "shaders/size_field.frag");

    struct Vertex {
        glm::vec2 pos;
        float value;
    };

    std::vector<Vertex> vertices;
    int w = grid_->get_width();
    int h = grid_->get_height();
    float cs = grid_->get_cell_size();
    glm::vec2 min_c = grid_->get_min_coords();
    const auto& field = grid_->get_target_size_field();

    for (int y = 0; y < h - 1; ++y) {
        for (int x = 0; x < w - 1; ++x) {
            // 每个格子创建两个三角形
            Vertex v0 = { min_c + glm::vec2(x * cs, y * cs),       field[y * w + x] };
            Vertex v1 = { min_c + glm::vec2((x + 1) * cs, y * cs),   field[y * w + x + 1] };
            Vertex v2 = { min_c + glm::vec2(x * cs, (y + 1) * cs),   field[(y + 1) * w + x] };
            Vertex v3 = { min_c + glm::vec2((x + 1) * cs, (y + 1) * cs), field[(y + 1) * w + x + 1] };
            vertices.push_back(v0); vertices.push_back(v1); vertices.push_back(v2);
            vertices.push_back(v1); vertices.push_back(v3); vertices.push_back(v2);
        }
    }

    glGenVertexArrays(1, &VAO_size_field_);
    glGenBuffers(1, &VBO_size_field_);
    glBindVertexArray(VAO_size_field_);
    glBindBuffer(GL_ARRAY_BUFFER, VBO_size_field_);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), vertices.data(), GL_STATIC_DRAW);
    // position
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);
    glEnableVertexAttribArray(0);
    // value (h_t)
    glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, value));
    glEnableVertexAttribArray(1);
    glBindVertexArray(0);
}
// --- 新增：静态键盘回调函数 ---

void Viewer::key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) 
{
    auto* viewer = static_cast<Viewer*>(glfwGetWindowUserPointer(window));
    if (viewer && action == GLFW_PRESS) { // 只在按下时触发一次
        if (key == GLFW_KEY_V) {
            // 循环切换视图: Particles -> Size Field -> Mesh
            if (!viewer->show_size_field_ && !viewer->show_mesh_) {
                viewer->show_size_field_ = true;
                std::cout << "View Mode: Size Field" << std::endl;
            }
            else if (viewer->show_size_field_) {
                viewer->show_size_field_ = false;
                viewer->show_mesh_ = true;
                std::cout << "View Mode: Triangle Mesh" << std::endl;
            }
            else {
                viewer->show_mesh_ = false;
                std::cout << "View Mode: Particles" << std::endl;
            }
        }
        if (key == GLFW_KEY_S) {
            viewer->save_particle_snapshot();
        }
        // --- 新增 C 键逻辑 ---
        if (key == GLFW_KEY_C) {
            // --- 关键修复：添加诊断性检查 ---
            if (!viewer->delaunay_generator_) {
                std::cerr << "Error: Delaunay Generator is not set!" << std::endl;
                return;
            }
            if (!viewer->sim2d_) {
                std::cerr << "Error: Simulation is not set!" << std::endl;
                return;
            }
            if (!viewer->boundary_) {
                std::cerr << "Error: Boundary is not set!" << std::endl;
                return;
            }
            
                std::cout << "Generating Delaunay Mesh..." << std::endl;
                viewer->delaunay_generator_->generate_mesh(viewer->sim2d_->get_particles(), *viewer->boundary_);
                viewer->update_mesh_buffers();
                viewer->show_mesh_ = true; // 生成后自动切换到网格视图
            
        }
    }
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

//void Viewer::set_mesh_generator2d(MeshGenerator2D* generator) {
//    generator2d_ = generator;
//    if (generator2d_) {
//        glGenVertexArrays(1, &VAO_mesh_);
//        glGenBuffers(1, &VBO_mesh_);
//        glGenBuffers(1, &EBO_mesh_);
//        glBindVertexArray(VAO_mesh_);
//        glBindBuffer(GL_ARRAY_BUFFER, VBO_mesh_);
//        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(glm::vec2), (void*)0);
//        glEnableVertexAttribArray(0);
//        glBindVertexArray(0);
//    }
//}


// --- 靜態回調函數 ---

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


void Viewer::run() {
    shader_ = new Shader("shaders/simple.vert", "shaders/simple.frag");
    point_shader_ = new Shader("shaders/point.vert", "shaders/point.frag");
    update_camera_vectors();
    main_loop();
}
void Viewer::init() {
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    window_ = glfwCreateWindow(width_, height_, title_.c_str(), nullptr, nullptr);
    glfwMakeContextCurrent(window_);
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) { return; }
    glfwSetWindowUserPointer(window_, this); // 关键：将this指针与窗口关联

    glfwSetWindowUserPointer(window_, this);
    glfwSetKeyCallback(window_, key_callback); // <-- 注册键盘回调
    glfwSetFramebufferSizeCallback(window_, framebuffer_size_callback);
    glfwSetMouseButtonCallback(window_, mouse_button_callback);
    glfwSetCursorPosCallback(window_, cursor_pos_callback);
    glfwSetScrollCallback(window_, scroll_callback);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_PROGRAM_POINT_SIZE);
}

void Viewer::update_camera_vectors() {
    float x = camera_target_.x + camera_radius_ * cos(glm::radians(camera_yaw_)) * cos(glm::radians(camera_pitch_));
    float y = camera_target_.y + camera_radius_ * sin(glm::radians(camera_pitch_));
    float z = camera_target_.z + camera_radius_ * sin(glm::radians(camera_yaw_)) * cos(glm::radians(camera_pitch_));
    camera_pos_ = glm::vec3(x, y, z);
}
void Viewer::update_particle_buffers() {
    if (!sim2d_ || sim2d_->get_particle_positions().empty()) return;
    glBindVertexArray(VAO_particles_);
    glBindBuffer(GL_ARRAY_BUFFER, VBO_particles_);
    glBufferData(GL_ARRAY_BUFFER, sim2d_->get_particle_positions().size() * sizeof(glm::vec2), sim2d_->get_particle_positions().data(), GL_DYNAMIC_DRAW);
    glBindVertexArray(0);
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