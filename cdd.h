#pragma once

#include <vector>
#include <cstdint>
#include <algorithm>
#include <cmath>
#include <vector>

namespace CDT
{
    // 2D aVertex
    template <typename T>
    struct V2d
    {
        T x, y;
        // The rest of the library doesn't use it, but it's handy for users
        std::size_t userData = 0;

        bool operator==(const V2d& other) const { return x == other.x && y == other.y; }
    };

    using VertInd = std::uint32_t;

    // An edge, identified by the vertex indices
    template <typename T>
    struct Edge
    {
        union
        {
            VertInd v[2];
            struct
            {
                VertInd v1, v2;
            };
        };
    };

    // A triangle, identified by the vertex indices
    template <typename T>
    struct Triangle
    {
        union
        {
            VertInd vertices[3];
            struct
            {
                VertInd v1, v2, v3;
            };
        };
    };

    // The main triangulation class
    template <typename T>
    class Triangulation
    {
    public:
        std::vector<V2d<T>> vertices;
        std::vector<Triangle<T>> triangles;

        void insertVertices(const std::vector<V2d<T>>& newVertices);
        void insertEdges(const std::vector<Edge<T>>& edges);
        void eraseOuterTrianglesAndHoles();

    private:
        std::vector<Edge<T>> internalEdges;

        void triangulate();
        std::vector<int> getAdjacency() const;
    };

    template <typename T>
    void Triangulation<T>::insertVertices(const std::vector<V2d<T>>& newVertices)
    {
        vertices.insert(vertices.end(), newVertices.begin(), newVertices.end());
        triangulate();
    }

    template <typename T>
    void Triangulation<T>::insertEdges(const std::vector<Edge<T>>& edges)
    {
        for (const auto& edge : edges)
        {
            const auto it = std::find_if(
                internalEdges.begin(),
                internalEdges.end(),
                [&edge](const Edge<T>& e)
                {
                    return (e.v1 == edge.v1 && e.v2 == edge.v2) || (e.v1 == edge.v2 && e.v2 == edge.v1);
                });
            if (it == internalEdges.end())
            {
                internalEdges.push_back(edge);
            }
        }
        triangulate();
    }

    template <typename T>
    void Triangulation<T>::eraseOuterTrianglesAndHoles()
    {
        if (internalEdges.empty())
        {
            return;
        }

        const auto adj = getAdjacency();
        std::vector<bool> isGood(triangles.size(), false);
        std::vector<VertInd> q;
        q.reserve(triangles.size());

        for (VertInd i = 0; i < triangles.size(); ++i)
        {
            bool isBorder = false;
            for (int j = 0; j < 3; ++j)
            {
                if (adj[i * 3 + j] == -1)
                {
                    isBorder = true;
                    break;
                }
            }

            if (isBorder)
            {
                q.push_back(i);
            }
        }

        std::size_t head = 0;
        while (head < q.size())
        {
            const VertInd triInd = q[head++];

            for (int i = 0; i < 3; ++i)
            {
                const VertInd v1 = triangles[triInd].vertices[i];
                const VertInd v2 = triangles[triInd].vertices[(i + 1) % 3];
                const auto it = std::find_if(
                    internalEdges.begin(),
                    internalEdges.end(),
                    [v1, v2](const Edge<T>& e)
                    {
                        return (e.v1 == v1 && e.v2 == v2) || (e.v1 == v2 && e.v2 == v1);
                    });

                if (it == internalEdges.end())
                {
                    const int neighbor = adj[triInd * 3 + i];
                    if (neighbor != -1 && !isGood[neighbor])
                    {
                        isGood[neighbor] = true;
                        q.push_back(neighbor);
                    }
                }
            }
        }

        const auto oldTriangles = triangles;
        triangles.clear();
        for (VertInd i = 0; i < oldTriangles.size(); ++i)
        {
            if (!isGood[i])
            {
                triangles.push_back(oldTriangles[i]);
            }
        }
    }

    template <typename T>
    void Triangulation<T>::triangulate()
    {
        if (vertices.size() < 3)
        {
            return;
        }
        const T minX = std::min_element(vertices.begin(), vertices.end(), [](const V2d<T>& a, const V2d<T>& b) { return a.x < b.x; })->x;
        const T minY = std::min_element(vertices.begin(), vertices.end(), [](const V2d<T>& a, const V2d<T>& b) { return a.y < b.y; })->y;
        const T maxX = std::max_element(vertices.begin(), vertices.end(), [](const V2d<T>& a, const V2d<T>& b) { return a.x < b.x; })->x;
        const T maxY = std::max_element(vertices.begin(), vertices.end(), [](const V2d<T>& a, const V2d<T>& b) { return a.y < b.y; })->y;
        const T dx = maxX - minX;
        const T dy = maxY - minY;
        const T deltaMax = std::max(dx, dy);
        const T midx = (minX + maxX) / 2;
        const T midy = (minY + maxY) / 2;

        const V2d<T> p1 = { midx - 20 * deltaMax, midy - deltaMax };
        const V2d<T> p2 = { midx, midy + 20 * deltaMax };
        const V2d<T> p3 = { midx + 20 * deltaMax, midy - deltaMax };

        const VertInd superInd1 = (VertInd)vertices.size();
        const VertInd superInd2 = superInd1 + 1;
        const VertInd superInd3 = superInd2 + 1;

        vertices.push_back(p1);
        vertices.push_back(p2);
        vertices.push_back(p3);

        triangles.clear();
        triangles.push_back({ superInd1, superInd2, superInd3 });

        for (VertInd i = 0; i < vertices.size() - 3; i++)
        {
            std::vector<Edge<T>> edges;
            std::vector<Triangle<T>> newTriangles;
            for (const auto& triangle : triangles)
            {
                const auto p1_ = vertices[triangle.v1];
                const auto p2_ = vertices[triangle.v2];
                const auto p3_ = vertices[triangle.v3];

                const T c_x = ((p1_.x * p1_.x + p1_.y * p1_.y) * (p2_.y - p3_.y) + (p2_.x * p2_.x + p2_.y * p2_.y) * (p3_.y - p1_.y) + (p3_.x * p3_.x + p3_.y * p3_.y) * (p1_.y - p2_.y)) / (2 * (p1_.x * (p2_.y - p3_.y) - p2_.x * (p1_.y - p3_.y) + p3_.x * (p1_.y - p2_.y)));
                const T c_y = ((p1_.x * p1_.x + p1_.y * p1_.y) * (p3_.x - p2_.x) + (p2_.x * p2_.x + p2_.y * p2_.y) * (p1_.x - p3_.x) + (p3_.x * p3_.x + p3_.y * p3_.y) * (p2_.x - p1_.x)) / (2 * (p1_.x * (p2_.y - p3_.y) - p2_.x * (p1_.y - p3_.y) + p3_.x * (p1_.y - p2_.y)));
                const T c_r = std::sqrt((p1_.x - c_x) * (p1_.x - c_x) + (p1_.y - c_y) * (p1_.y - c_y));

                if (std::sqrt(std::pow(vertices[i].x - c_x, 2) + std::pow(vertices[i].y - c_y, 2)) < c_r)
                {
                    edges.push_back({ triangle.v1, triangle.v2 });
                    edges.push_back({ triangle.v2, triangle.v3 });
                    edges.push_back({ triangle.v3, triangle.v1 });
                }
                else
                {
                    newTriangles.push_back(triangle);
                }
            }

            for (auto it1 = edges.begin(); it1 != edges.end(); ++it1)
            {
                for (auto it2 = edges.begin(); it2 != edges.end(); ++it2)
                {
                    if (it1 == it2)
                    {
                        continue;
                    }
                    if ((it1->v1 == it2->v2) && (it1->v2 == it2->v1))
                    {
                        it1->v1 = (VertInd)-1;
                        it1->v2 = (VertInd)-1;
                        it2->v1 = (VertInd)-1;
                        it2->v2 = (VertInd)-1;
                    }
                }
            }

            for (const auto& edge : edges)
            {
                if (edge.v1 != (VertInd)-1 && edge.v2 != (VertInd)-1)
                {
                    newTriangles.push_back({ edge.v1, edge.v2, i });
                }
            }
            triangles = newTriangles;
        }

        const auto oldTriangles = triangles;
        triangles.clear();

        for (const auto& triangle : oldTriangles)
        {
            if (triangle.v1 < superInd1 && triangle.v2 < superInd1 && triangle.v3 < superInd1)
            {
                triangles.push_back(triangle);
            }
        }

        vertices.erase(vertices.end() - 3, vertices.end());
    }

    template <typename T>
    std::vector<int> Triangulation<T>::getAdjacency() const
    {
        std::vector<int> adj(triangles.size() * 3, -1);
        for (VertInd i = 0; i < triangles.size(); ++i)
        {
            for (VertInd j = i + 1; j < triangles.size(); ++j)
            {
                const auto& t1 = triangles[i];
                const auto& t2 = triangles[j];
                int shared = 0;
                int t1E = -1;
                int t2E = -1;
                if ((t1.v1 == t2.v1 && t1.v2 == t2.v2) || (t1.v1 == t2.v2 && t1.v2 == t2.v1)) { shared++; t1E = 2; t2E = 2; }
                if ((t1.v2 == t2.v1 && t1.v3 == t2.v2) || (t1.v2 == t2.v2 && t1.v3 == t2.v1)) { shared++; t1E = 0; t2E = 2; }
                if ((t1.v3 == t2.v1 && t1.v1 == t2.v2) || (t1.v3 == t2.v2 && t1.v1 == t2.v1)) { shared++; t1E = 1; t2E = 2; }

                if ((t1.v1 == t2.v2 && t1.v2 == t2.v3) || (t1.v1 == t2.v3 && t1.v2 == t2.v2)) { shared++; t1E = 2; t2E = 0; }
                if ((t1.v2 == t2.v2 && t1.v3 == t2.v3) || (t1.v2 == t2.v3 && t1.v3 == t2.v2)) { shared++; t1E = 0; t2E = 0; }
                if ((t1.v3 == t2.v2 && t1.v1 == t2.v3) || (t1.v3 == t2.v3 && t1.v1 == t2.v2)) { shared++; t1E = 1; t2E = 0; }

                if ((t1.v1 == t2.v3 && t1.v2 == t2.v1) || (t1.v1 == t2.v1 && t1.v2 == t2.v3)) { shared++; t1E = 2; t2E = 1; }
                if ((t1.v2 == t2.v3 && t1.v3 == t2.v1) || (t1.v2 == t2.v1 && t1.v3 == t2.v3)) { shared++; t1E = 0; t2E = 1; }
                if ((t1.v3 == t2.v3 && t1.v1 == t2.v1) || (t1.v3 == t2.v1 && t1.v1 == t2.v3)) { shared++; t1E = 1; t2E = 1; }

                if (shared == 2)
                {
                    adj[i * 3 + t1E] = j;
                    adj[j * 3 + t2E] = i;
                }
            }
        }
        return adj;
    }
}