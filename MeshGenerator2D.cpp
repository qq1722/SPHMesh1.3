#include "MeshGenerator2D.h"
#include "Simulation2D.h"
#include "Boundary.h"
#include <unordered_map>
#include <unordered_set>
#include <glm/glm.hpp>
#include <algorithm>
#include <vector>

struct ivec2_hash {
    std::size_t operator()(const glm::ivec2& v) const {
        return std::hash<int>()(v.x) ^ (std::hash<int>()(v.y) << 1);
    }
};
struct pair_hash {
    template <class T1, class T2>
    std::size_t operator () (const std::pair<T1, T2>& p) const {
        auto h1 = std::hash<T1>{}(p.first);
        auto h2 = std::hash<T2>{}(p.second);
        return h1 ^ (h2 << 1);
    }
};

void MeshGenerator2D::generate(const Simulation2D& sim, const Boundary& boundary, float grid_size) {
    vertices_.clear();
    quads_.clear();
    triangles_.clear();

    const auto& particles = sim.get_particle_positions();
    if (particles.empty()) return;
    vertices_ = particles;

    std::unordered_map<glm::ivec2, unsigned int, ivec2_hash> grid_to_vertex_id_map;
    for (unsigned int i = 0; i < vertices_.size(); ++i) {
        glm::ivec2 grid_index = { (int)round(vertices_[i].x / grid_size), (int)round(vertices_[i].y / grid_size) };
        grid_to_vertex_id_map[grid_index] = i;
    }

    std::unordered_map<std::pair<unsigned int, unsigned int>, int, pair_hash> edge_counts;
    for (const auto& pair : grid_to_vertex_id_map) {
        const auto& p0_idx = pair.first;
        auto it_p1 = grid_to_vertex_id_map.find({ p0_idx.x + 1, p0_idx.y });
        auto it_p2 = grid_to_vertex_id_map.find({ p0_idx.x + 1, p0_idx.y + 1 });
        auto it_p3 = grid_to_vertex_id_map.find({ p0_idx.x, p0_idx.y + 1 });

        if (it_p1 != grid_to_vertex_id_map.end() && it_p2 != grid_to_vertex_id_map.end() && it_p3 != grid_to_vertex_id_map.end()) {
            unsigned int v0 = pair.second, v1 = it_p1->second, v2 = it_p2->second, v3 = it_p3->second;
            quads_.push_back({ v0, v1, v2, v3 });
            edge_counts[{std::min(v0, v1), std::max(v0, v1)}]++;
            edge_counts[{std::min(v1, v2), std::max(v1, v2)}]++;
            edge_counts[{std::min(v2, v3), std::max(v2, v3)}]++;
            edge_counts[{std::min(v3, v0), std::max(v3, v0)}]++;
        }
    }

    std::unordered_set<unsigned int> used_vertices;
    for (const auto& q : quads_) {
        used_vertices.insert({ q.v0, q.v1, q.v2, q.v3 });
    }

    std::vector<unsigned int> isolated_vertices;
    for (unsigned int i = 0; i < particles.size(); ++i) {
        if (used_vertices.find(i) == used_vertices.end()) {
            isolated_vertices.push_back(i);
        }
    }

    std::vector<std::pair<unsigned int, unsigned int>> inner_border_edges;
    for (const auto& pair : edge_counts) {
        if (pair.second == 1) {
            inner_border_edges.push_back(pair.first);
        }
    }

    if (inner_border_edges.empty()) return;

    for (unsigned int isolated_id : isolated_vertices) {
        const auto& isolated_pos = vertices_[isolated_id];
        float min_dist_sq = FLT_MAX;
        std::pair<unsigned int, unsigned int> closest_edge = { 0,0 };

        for (const auto& edge : inner_border_edges) {
            glm::vec2 edge_midpoint = (vertices_[edge.first] + vertices_[edge.second]) * 0.5f;
            float dist_sq = glm::dot(isolated_pos - edge_midpoint, isolated_pos - edge_midpoint);
            if (dist_sq < min_dist_sq) {
                min_dist_sq = dist_sq;
                closest_edge = edge;
            }
        }

        if (closest_edge.first != 0 || closest_edge.second != 0) {
            triangles_.push_back({ isolated_id, closest_edge.first, closest_edge.second });
        }
    }
}
