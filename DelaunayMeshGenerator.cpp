#include "DelaunayMeshGenerator.h"
#include "CDT.h" // <-- �����µ�ͷ�ļ�
#include <iostream>

void DelaunayMeshGenerator::generate_mesh(const std::vector<Simulation2D::Particle>& particles, const Boundary& boundary) {
    vertices_.clear();
    triangles_.clear();

    if (particles.empty()) return;

    // 1. ���� CDT ����
    CDT::Triangulation<float> cdt;

    // 2. �����������Ӷ���
    std::vector<CDT::V2d<float>> cdt_vertices;
    cdt_vertices.reserve(particles.size());
    for (const auto& p : particles) {
        cdt_vertices.push_back({ p.position.x, p.position.y });
    }
    cdt.insertVertices(cdt_vertices);

    // 3. ����߽���ΪԼ����
    const auto& boundary_vertices = boundary.get_vertices();
    std::vector<CDT::Edge> cdt_edges;
    cdt_edges.reserve(boundary_vertices.size());
    for (size_t i = 0; i < boundary_vertices.size(); ++i) {
        // CDT �Ķ��������Ǵ� 0 ��ʼ��
        // ע�⣺�������Ǽ���߽綥��������ӵ㼯��ǰN����
        // һ����³���ķ������Ƚ��߽綥��Ҳ���������б�
        // ��Ϊ����������Ǽ���߽��Ѿ������Ӳ�������
    }
    // CDT���ڲ��붥��ʱ���Զ�����߽磬����ֻ��Ҫ����Ƴ��ⲿ������
    // cdt.insertEdges(cdt_edges);


    // 4. ִ�����ǻ� (�Ƴ������ṹ���������������)
   // cdt.eraseSuperfluousTriangles(boundary_vertices, false);
    cdt.eraseSuperTriangle();
    cdt.eraseOuterTriangles();

    // 5. ��ȡ���
    const auto& result_vertices = cdt.vertices;
    vertices_.reserve(result_vertices.size());
    for (const auto& v : result_vertices) {
        vertices_.emplace_back(v.x, v.y);
    }

    const auto& result_triangles = cdt.triangles;
    triangles_.reserve(result_triangles.size());
    for (const auto& t : result_triangles) {
        triangles_.push_back({
            static_cast<unsigned int>(t.vertices[0]),
            static_cast<unsigned int>(t.vertices[1]),
            static_cast<unsigned int>(t.vertices[2])
            });
    }

    std::cout << "CDT generated: " << vertices_.size() << " vertices, " << triangles_.size() << " triangles." << std::endl;
}