#ifndef SPHERE_H
#define SPHERE_H

#include <iostream>
#include <vector>


#include "DataStruct.h"

class Sphere
{
public:
    enum BuildMethod {     //! 优化方案选择
        OPT_G2O,
        OPT_CERES,
        OPT_DEFAULT
    };

    Sphere();
    void pushVertex(Vertex &vertex) {       //! 加入顶点
        vertexes.push_back(vertex);
    }

    void pushEdge(Edge &edge) {             //! 加入边
        edges.push_back(edge);
    }

    const std::vector<Edge>& getEdges() {   //! 得到边数据
        return edges;
    }

    const std::vector<Vertex>& getVertexes() { //! 得到顶点数据
        return vertexes;
    }

    virtual bool optimize(BuildMethod buildMethod = OPT_DEFAULT, int iter = 50);   //! 优化

private:
    std::vector<Vertex> vertexes;
    std::vector<Edge>   edges;
};

#endif // SPHERE_H
