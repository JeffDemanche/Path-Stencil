#ifndef PATHTRACER_H
#define PATHTRACER_H

#include <QImage>

#include <iostream>

#include <Eigen/Dense>

#include "scene/scene.h"

class PathTracer
{
public:
    PathTracer(int width, int height);

    void traceScene(QRgb *imageData, const Scene &scene);

private:
    const double pi = 3.1415926535897;

    int m_width, m_height;

    float randFloat();

    void toneMap(QRgb *imageData, std::vector<Eigen::Vector3f> &intensityValues);

    Eigen::Vector3f brdfDiffuse(Eigen::Vector3f w_i, Eigen::Vector3f w_o, const tinyobj::material_t& mat);

    Eigen::Vector3f tracePixel(int x, int y, const Scene &scene, const Eigen::Matrix4f &invViewMatrix);
    Eigen::Vector3f traceRay(const Ray& r, const Scene &scene, int depth = 0);
};

#endif // PATHTRACER_H
