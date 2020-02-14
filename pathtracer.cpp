#include "pathtracer.h"

#include <stdlib.h>

#include <iostream>

#include <Eigen/Dense>

#include <util/CS123Common.h>

#include <math.h>

using namespace Eigen;

PathTracer::PathTracer(int width, int height)
    : m_width(width), m_height(height)
{
}

/**
 * @return A float between 0 and 1.
 */
float PathTracer::randFloat()
{
    return static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
}

void PathTracer::traceScene(QRgb *imageData, const Scene& scene)
{
    std::vector<Vector3f> intensityValues(m_width * m_height);
    Matrix4f invViewMat = (scene.getCamera().getScaleMatrix() * scene.getCamera().getViewMatrix()).inverse();
    for(int y = 0; y < m_height; ++y) {
        //#pragma omp parallel for
        for(int x = 0; x < m_width; ++x) {
            int offset = x + (y * m_width);
            intensityValues[offset] = tracePixel(x, y, scene, invViewMat);
        }
    }

    toneMap(imageData, intensityValues);
}

Vector3f PathTracer::tracePixel(int x, int y, const Scene& scene, const Matrix4f &invViewMatrix)
{
    int numSamples = 200;
    Vector3f pixelVal;

    for(int sample = 0; sample < numSamples; sample++) {
        float nextXSampleOffset = randFloat();
        float nextYSampleOffset = randFloat();

        Vector3f p(0, 0, 0);
        Vector3f d((2.f * (x + nextXSampleOffset) / m_width) - 1, 1 - (2.f * (y + nextYSampleOffset) / m_height), -1);
        d.normalize();

        Ray r(p, d);
        r = r.transform(invViewMatrix);

        pixelVal += traceRay(r, scene);
    }

    return pixelVal / static_cast<float>(numSamples);
}

Vector3f PathTracer::brdfDiffuse(Eigen::Vector3f w_i, Eigen::Vector3f w_o, const tinyobj::material_t& mat)
{
    return Vector3f(mat.diffuse[0] / pi, mat.diffuse[1] / pi, mat.diffuse[2] / pi);
}

Vector3f PathTracer::traceRay(const Ray& r, const Scene& scene, int depth)
{
    IntersectionInfo i;
    Ray ray(r);
    if(scene.getIntersection(ray, &i)) {
          //** Example code for accessing materials provided by a .mtl file **
        const Triangle *t = static_cast<const Triangle *>(i.data);//Get the triangle in the mesh that was intersected
        const tinyobj::material_t& mat = t->getMaterial();//Get the material of the triangle from the mesh
        //const tinyobj::real_t *d = mat.diffuse;//Diffuse color as array of floats
        //const std::string diffuseTex = mat.diffuse_texname;//Diffuse texture name

        // Emissive contribution.
        Vector3f L = Vector3f(mat.emission[0], mat.emission[1], mat.emission[2]);

        float pdf_rr = 0.875;

        // TODO should ray depth culling be factored into the probability.
        if (randFloat() < pdf_rr) {

            float e_1 = randFloat();
            float e_2 = randFloat();

            float nextPhi = 2 * pi * e_1;
            float nextTheta = acos(1.0 - e_2);

            float x = sin(nextTheta) * cos(nextPhi);
            float y = sin(nextTheta) * sin(nextPhi);
            float z = cos(nextTheta);

            float pdf_omega = 1.0 / (2.0 * pi);

            Vector3f w_i = Vector3f(x, y, z);
            w_i.normalize();

            // reflected radiance * brdf * dot of angle from surface normal.
            Vector3f trace = traceRay(Ray(i.hit, w_i), scene, depth + 1);

            Vector3f integrand = trace.cwiseProduct(brdfDiffuse(w_i, ray.d, t->getMaterial()))
                                 * w_i.dot(i.object->getNormal(i));
            integrand /= (pdf_rr * pdf_omega);

            L += integrand;
        }

        return L;
    } else {
        // No intersection.
        return Vector3f(0, 0, 0);
    }
}

void PathTracer::toneMap(QRgb *imageData, std::vector<Vector3f> &intensityValues) {
    for(int y = 0; y < m_height; ++y) {
        for(int x = 0; x < m_width; ++x) {
            int offset = x + (y * m_width);

            // Reinhard operator
            float r = intensityValues[offset].x() / (1.0 - intensityValues[offset].x());
            float g = intensityValues[offset].y() / (1.0 - intensityValues[offset].y());
            float b = intensityValues[offset].z() / (1.0 - intensityValues[offset].z());

            QRgb output = qRgb(r * 255.0, g * 255.0, b * 255.0);

            imageData[offset] = output;
        }
    }

}
