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
    Eigen::Vector3f brdfPhong(Eigen::Vector3f w_i, Eigen::Vector3f w_o, Eigen::Vector3f normal, const tinyobj::material_t& mat);

    float fresnel(Eigen::Vector3f w_i, Eigen::Vector3f normal, float ior);

    Eigen::Vector3f directLightEstimator(Triangle* triangle, Eigen::Vector3f p, Eigen::Vector3f normal, const Scene& scene, int numSamples);

    Eigen::Matrix3f rotationMatrix(Eigen::Vector3f a, Eigen::Vector3f b);

    Eigen::Vector3f tracePixel(int x, int y, const Scene &scene, const Eigen::Matrix4f &invViewMatrix);
    Eigen::Vector3f traceRay(int x, int y, const Ray& r, const Scene &scene, int depth = 0);

    Eigen::Vector3f reflect(Eigen::Vector3f w_i, Eigen::Vector3f normal);
    Eigen::Vector3f refract(Eigen::Vector3f w_i, Eigen::Vector3f normal, float materialIor);
};

#endif // PATHTRACER_H
