#include "mesh.h"


#include <iostream>

using namespace Eigen;
using namespace std;

void Mesh::init(const std::vector<Vector3f> &vertices,
           const std::vector<Vector3f> &normals,
           const std::vector<Vector2f> &uvs,
           const std::vector<Vector3f> &colors,
           const std::vector<Vector3i> &faces,
           const std::vector<int> &materialIds,
           const std::vector<tinyobj::material_t> &materials)
{
    _vertices = vertices;
    _normals = normals;
    _colors = colors;
    _uvs = uvs;
    _faces = faces;
    _materialIds = materialIds;
    _materials = materials;
    calculateMeshStats();
    createMeshBVH();
}

Mesh::~Mesh()
{
    delete _meshBvh;
    delete _objects;
    delete[] _triangles;
}

bool Mesh::getIntersection(const Ray &ray, IntersectionInfo *intersection) const
{
//    std::cout << "RAY" << std::endl;
//    std::cout << ray.getO().transpose() << " " << ray.getD().transpose() << std::endl;
    Ray r(ray.transform(transform));
//    std::cout << r.getO().transpose() << " " << r.getD().transpose() << std::endl;
    IntersectionInfo i;
    bool col = _meshBvh->getIntersection(r, &i, false);
    if(col) {
        intersection->t = i.t;
        intersection->object = this;
        intersection->data = i.object;

        return true;
    }
    return false;
}

Vector3f Mesh::getNormal(const IntersectionInfo &I) const
{
    return static_cast<const Object *>(I.data)->getNormal(I);
}

BBox Mesh::getBBox() const
{
    return _bbox;
}

Vector3f Mesh::getCentroid() const
{
    return _centroid;
}

void Mesh::calculateMeshStats()
{
    _bbox.setP(_vertices[0]);
    for(auto v : _vertices) {
        _centroid += v;
        _bbox.expandToInclude(v);
    }
    _centroid /= _vertices.size();
}

void Mesh::createMeshBVH()
{
    _triangles = new Triangle[_faces.size()];
    _objects = new std::vector<Object *>;
    _objects->resize(_faces.size());
    for(unsigned int i = 0; i < _faces.size(); ++i) {
        Vector3i face = _faces[i];
        Vector3f v1 = _vertices[face(0)];
        Vector3f v2 = _vertices[face(1)];
        Vector3f v3 = _vertices[face(2)];
        Vector3f n1 = _normals[face[0]];
        Vector3f n2 = _normals[face[1]];
        Vector3f n3 = _normals[face[2]];
        _triangles[i] = Triangle(v1, v2, v3, n1, n2, n3);
        _triangles[i].transform = transform;
        (*_objects)[i] = &_triangles[i];
    }

    _meshBvh = new BVH(_objects);
}
