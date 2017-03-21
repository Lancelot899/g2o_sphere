#ifndef SPHERE_H
#define SPHERE_H

#include <iostream>
#include <vector>


#include "DataStruct.h"

class Sphere
{
public:
    Sphere();
    void pushVertex(Vertex &vertex) {
        vertexes.push_back(vertex);
    }

    void pushEdge(Edge &edge) {
        edges.push_back(edge);
    }

    const std::vector<Edge>& getEdges() {
        return edges;
    }

    const std::vector<Vertex>& getVertexes() {
        return vertexes;
    }

    virtual bool optimize(int iter = 50);

private:
    std::vector<Vertex> vertexes;
    std::vector<Edge>   edges;
};

#endif // SPHERE_H
