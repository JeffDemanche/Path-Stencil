#include "pathtracer.h"

#include <stdlib.h>

#include <iostream>

#include <Eigen/Dense>

#include <util/CS123Common.h>
#include <util/hdrloader.h>

#include <math.h>

using namespace Eigen;

PathTracer::PathTracer(int width, int height, HDRLoaderResult hdrResult, float aperture, float focalLength)
    : m_width(width), m_height(height), m_aperture(aperture), m_focalLength(focalLength), m_hdr(hdrResult)
{
}

/**
 * @return A float between 0 and 1.
 */
float PathTracer::randFloat()
{
    return static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
}

void PathTracer::traceScene(QRgb *imageData, const Scene& scene, int samples)
{
    int currentPercent = 0;

    std::vector<Vector3f> intensityValues(m_width * m_height);
    Matrix4f invViewMat = (scene.getCamera().getScaleMatrix() * scene.getCamera().getViewMatrix()).inverse();
    for(int y = 0; y < m_height; ++y) {
        //#pragma omp parallel for
        for(int x = 0; x < m_width; ++x) {
            int offset = x + (y * m_width);

            float percent = 100 * static_cast<float>(offset) / static_cast<float>(m_width * m_height);
            if (static_cast<int>(percent) > currentPercent) {
                currentPercent = static_cast<int>(percent);
                printf("Rendering: %i%% \n", currentPercent);
            }

            intensityValues[offset] = tracePixel(x, y, scene, invViewMat, samples);
        }
    }

    toneMap(imageData, intensityValues);
}

Vector3f PathTracer::tracePixel(int x, int y, const Scene& scene, const Matrix4f &invViewMatrix, int samples)
{
    Vector3f pixelVal;

    for(int sample = 0; sample < samples; sample++) {
        float nextXSampleOffset = randFloat() - 0.5;
        float nextYSampleOffset = randFloat() - 0.5;

        float apertureRadius = m_aperture;
        float focalDistance = m_focalLength;

        Vector3f p(0, 0, 0);
        Vector3f d((2.f * (x + nextXSampleOffset) / m_width) - 1, 1 - (2.f * (y + nextYSampleOffset) / m_height), -1);
        d.normalize();

        // DoF
        float rand1 = randFloat();
        float rand2 = randFloat();
        float radius1 = 2.0 * pi * rand1;
        float radius2 = rand2;
        Vector3f offset = Vector3f(cos(radius1) * radius2, sin(radius1) * radius2, 0.0) * apertureRadius;

        Vector3f p1 = p + d * (focalDistance);

        p = p + offset;
        d = (p1 - p).normalized();

        Ray r(p, d);
        r = r.transform(invViewMatrix);

        pixelVal += traceRay(r, scene);
    }

    return pixelVal / static_cast<float>(samples);
}

Vector3f PathTracer::brdfDiffuse(Eigen::Vector3f w_i, Eigen::Vector3f w_o, const tinyobj::material_t& mat)
{
    return Vector3f(mat.diffuse[0], mat.diffuse[1], mat.diffuse[2]) / pi;
}

Vector3f PathTracer::brdfPhong(Vector3f w_i, Vector3f w_o, Vector3f normal, const tinyobj::material_t& mat)
{
    float n = mat.shininess;
    Vector3f p_s = Vector3f(mat.specular[0], mat.specular[1], mat.specular[2]);
    Vector3f refl_w_i = w_i - 2.0 * (w_i.dot(normal)) * normal;
    return p_s * ((n + 2) / (2 * pi)) * pow(refl_w_i.dot(w_o), n);
}

float PathTracer::fresnel(Vector3f w_i, Vector3f normal, float ior)
{
    float n_i = 1;
    float n_t = ior;
    float cos_theta_i = w_i.dot(normal);
    if (cos_theta_i < 0) {
        cos_theta_i = -cos_theta_i;
    } else {
        n_i = ior;
        n_t = 1;
    }

    float R_0 = pow((n_i - n_t) / (n_i + n_t), 2);
    float R_theta_i = R_0 + (1.0 - R_0) * pow(1.0 - cos_theta_i, 5);
    return R_theta_i;
}

Vector3f PathTracer::directLightEstimator(Triangle* triangle, Vector3f p, Vector3f normal, const Scene& scene, int numSamples)
{
    tinyobj::material_t emisiveMat = triangle->getMaterial();
    Vector3f emission = Vector3f(emisiveMat.emission[0] * 5, emisiveMat.emission[1] * 5, emisiveMat.emission[2] * 5);

    Vector3f sum = Vector3f(0, 0, 0);

    for(int sample = 0; sample < numSamples; sample++) {
        float r_1 = randFloat();
        float r_2 = randFloat();
        std::vector<Vector3f*> v = triangle->getVerts();
        Vector3f A = *v.at(0);
        Vector3f B = *v.at(1);
        Vector3f C = *v.at(2);

        // p' is a point on the triangle sampled uniformly.
        // Uniform sampling from https://math.stackexchange.com/questions/18686/uniform-random-point-in-triangle.
        Vector3f p_prime = (1 - sqrt(r_1)) * A
                           + (sqrt(r_1) * (1 - r_2)) * B
                           + (r_2 * sqrt(r_1)) * C;

        Vector3f w = p_prime - p;
        Vector3f w_prime = p - p_prime;

        IntersectionInfo i_scene;
        Ray r(p, w);
        if (scene.getIntersection(r, &i_scene) && (i_scene.hit - p_prime).norm() < 0.001) {
            float cos_theta = w.dot(normal) / w.norm();
            float cos_theta_prime = w_prime.dot(triangle->getNormal(p_prime)) / w_prime.norm();

            cos_theta = 0 > cos_theta ? 0 : cos_theta;
            cos_theta_prime = 0 > cos_theta_prime ? 0 : cos_theta_prime;

            // TODO is this correct or should it account for angle?
            Vector3f L_o = emission;
            float finalTerm = (cos_theta * cos_theta_prime) / pow((p - p_prime).norm(), 2);

            sum += emission * finalTerm;
       }
       else {
            //std::cout << "Direct light sample didn't intersect tri." << std::endl;
       }
    }
    return (triangle->getArea() / numSamples) * sum;
}

Vector3f PathTracer::traceRay(const Ray& r, const Scene& scene, int depth)
{
    IntersectionInfo i;
    Ray ray(r);
    if(scene.getIntersection(ray, &i)) {
          //** Example code for accessing materials provided by a .mtl file **
        const Triangle *t = static_cast<const Triangle *>(i.data);//Get the triangle in the mesh that was intersected
        const tinyobj::material_t& mat = t->getMaterial();//Get the material of the triangle from the mesh

        float e_1 = randFloat();
        float e_2 = randFloat();

        float nextPhi = 2 * pi * e_1;
        float nextTheta = acos(1.0 - e_2);

        float x = sin(nextTheta) * cos(nextPhi);
        float y = sin(nextTheta) * sin(nextPhi);
        float z = cos(nextTheta);

        Vector3f iNormal = t->getNormal(i);

        Vector3f w_o = rotationMatrix(Vector3f(0, 0, 1), iNormal) * Vector3f(x, y, z);
        w_o.normalize();

        float pdf_omega = 1.0 / (2.0 * pi);

        float pdf_rr = 0.75;

        bool mirror = false;
        bool refraction = false;

        Vector3f brdf;
        if (mat.illum == 5) {
            mirror = true;
        }
        else if (mat.illum == 7) {
            refraction = true;
        }
        else if (mat.specular[0] + mat.specular[1] + mat.specular[2] > 0.0) {
            brdf = brdfPhong(w_o, ray.d, iNormal, t->getMaterial());
        } else {
            brdf = brdfDiffuse(w_o, ray.d, t->getMaterial());
        }

        Vector3f L = Vector3f(0, 0, 0);
        // Iterate over light meshes in scene
        for (unsigned int j = 0; j < scene.getEmissiveTris()->size(); j++) {
            Triangle* tri = scene.getEmissiveTris()->at(j);

            if (!mirror && !refraction)
                L += directLightEstimator(tri, i.hit, iNormal, scene, 1).cwiseProduct(brdf);
        }

        if (randFloat() < pdf_rr) {
            // reflected radiance * brdf * dot of angle from surface normal.
            if (mirror) {
                Vector3f reflectVector = reflect(r.d, iNormal);
                // Note depth set to 0 here.
                Vector3f integrand = traceRay(Ray(i.hit, reflectVector), scene, 0);
                integrand /= pdf_rr;
                L += integrand;
            }
            else if (refraction) {
                Vector3f refractVector = refract(r.d, iNormal, mat.ior);

                float fresnel_R_theta;
                fresnel_R_theta = fresnel(r.d, iNormal, mat.ior);

                if (randFloat() > fresnel_R_theta) {
                    // Note depth set to 0 here.
                    Vector3f integrand = traceRay(Ray(i.hit, refractVector), scene, 0);
                    integrand /= pdf_rr;
                    L += integrand;
                } else {
                    Vector3f reflectVector = reflect(r.d, iNormal);
                    // Note depth set to 0 here.
                    Vector3f integrand = traceRay(Ray(i.hit, reflectVector), scene, 0);
                    integrand /= pdf_rr;
                    L += integrand;
                }
            }
            else {
                Vector3f trace = traceRay(Ray(i.hit, w_o), scene, depth + 1);
                Vector3f integrand = trace.cwiseProduct(brdf);
                integrand *= w_o.dot(iNormal);
                integrand /= (pdf_rr * pdf_omega);

                L += integrand;
            }
        }

        if (depth == 0) {
            L += Vector3f(mat.emission[0], mat.emission[1], mat.emission[2]) * 5;
        }

        return L;
    } else {
        // No intersection.
        return sampleHDR(r.d);
    }
}

Matrix3f PathTracer::rotationMatrix(Vector3f a, Vector3f b) {
    if (a == b) {
        return Eigen::Matrix3f::Identity();
    }

    Vector3f v = a.cross(b);
    float c = a.dot(b);
    float s = v.norm();
    assert (s != 0);

    Matrix3f sscpm;
    sscpm <<      0, -v.z(),  v.y(),
              v.z(),      0, -v.x(),
             -v.y(),  v.x(),      0;
    return Eigen::Matrix3f::Identity() + sscpm + (sscpm * sscpm) * ((1.0 - c) / pow(s, 2));
}

Vector3f PathTracer::reflect(Vector3f w_i, Vector3f normal) {
    Vector3f reflectVector = w_i - 2.0 * (w_i.dot(normal)) * normal;

    return reflectVector;
}

Vector3f PathTracer::refract(Vector3f w_i, Vector3f normal, float materialIor) {
    // I drew from here for help:
    // https://www.scratchapixel.com/lessons/3d-basic-rendering/introduction-to-shading/reflection-refraction-fresnel
    float ni_nt = 1 / materialIor;

    Vector3f refNormal = normal;

    float cos_theta_i = w_i.dot(normal);
    if (cos_theta_i < 0) {
        cos_theta_i = -cos_theta_i;
    } else {
        // This is where the ray is inside the object and hitting its surface from inside.
        ni_nt = materialIor; // 1 / ni_nt
        refNormal = -refNormal;
    }
    float cos_theta_t = sqrt(1.0 - ni_nt * ni_nt * (1.0 - pow(cos_theta_i, 2)));
    Vector3f refractVector = ni_nt * w_i + (ni_nt * cos_theta_i - cos_theta_t) * refNormal;
    refractVector.normalize();

    return refractVector;
}

void PathTracer::toneMap(QRgb *imageData, std::vector<Vector3f> &intensityValues) {
    for(int y = 0; y < m_height; ++y) {
        for(int x = 0; x < m_width; ++x) {
            int offset = x + (y * m_width);

            // Reinhard operator
            float r = intensityValues[offset].x() / (1.0 + intensityValues[offset].x());
            float g = intensityValues[offset].y() / (1.0 + intensityValues[offset].y());
            float b = intensityValues[offset].z() / (1.0 + intensityValues[offset].z());

            QRgb output = qRgb(r * 255.0, g * 255.0, b * 255.0);

            imageData[offset] = output;
        }
    }

}

Vector3f PathTracer::sampleHDR(Vector3f direction) {
    if (m_hdr.width == 0 || m_hdr.height == 0) {
        return Vector3f(0, 0, 0);
    }

    direction.normalize();

    float r = (1.0 / pi) * acos(direction.z()) / sqrt(direction.x() * direction.x() + direction.y() * direction.y());
    float u = direction.x() * r;
    float v = direction.y() * r;

    int x = (unsigned int) floor(((u + 1.0) / 2.0) * m_hdr.width);
    int y = (unsigned int) floor(((v + 1.0) / 2.0) * m_hdr.height);
    // Flip vertically.
    y = m_hdr.height - y - 1;

    int offset = ((y * m_hdr.width) + x) * 3;

    return 10 * Vector3f(m_hdr.cols[offset], m_hdr.cols[offset + 1], m_hdr.cols[offset + 2]);
}
