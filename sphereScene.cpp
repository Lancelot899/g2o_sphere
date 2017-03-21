#include "sphereScene.h"
#include "Sphere.h"

SphereScene::SphereScene(QWidget* parent, const QGLWidget* shareWidget, Qt::WindowFlags flags) :
    QGLViewer(parent, shareWidget, flags)
{
    isInit = false;
}

void SphereScene::draw()
{
    if(isInit == true) {
        setSceneRadius(350);
        const std::vector<Vertex>& vertexes = sphere->getVertexes();
        const std::vector<Edge>&   edges    = sphere->getEdges();

        glBegin(GL_POINTS);
        glColor3f(1.0f, 0.5f, 1.0f);
        for(size_t i = 0; i < vertexes.size(); i++) {
            const Vertex &vertex = vertexes[i];
            auto pose =  Sophus::SE3d::exp(vertex.pose);
            glVertex3d(pose.translation()[0], pose.translation()[1], pose.translation()[2]);        }
        glEnd();

        glBegin(GL_LINES);
        for(size_t i = 0; i < edges.size(); ++i) {
            const Edge &edge = edges[i];
            glColor3f(1, 0 ,0);
            auto pose_i = Sophus::SE3d::exp(vertexes[edge.i].pose);
            auto pose_j = Sophus::SE3d::exp(vertexes[edge.j].pose);
            //if(i == 8645) std::cout << vertexes[edge.i].pose << std::endl;
            glVertex3d(pose_i.translation()[0],
                    pose_i.translation()[1],
                    pose_i.translation()[2]);
            glColor3f(0,1,0);
            glVertex3d(pose_j.translation()[0],
                    pose_j.translation()[1],
                    pose_j.translation()[2]);
        }
        glEnd();
    }
}

void SphereScene::init()
{
    restoreStateFromFile();
    glDisable(GL_LIGHTING);
    glPointSize(3.0);
    setGridIsDrawn();
}
