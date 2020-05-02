
#ifndef PATHTRACER_H
#define PATHTRACER_H

#include <QImage>

#include "scene/scene.h"

#include <hdrloader/hdrloader.h>

#define START_P 0.8f        //initial (stop) probability (for the first 5 bounces)
#define EPSILON 0.0001f       //epsilon term (direct lighting -- to check for light intersection vs occlusion)
#define BASE_X 2
#define BASE_Y 3
#define GRID_DIM 10         //dimension of stratified sampling grid

using namespace Eigen;
using namespace std;

enum MODE {
    INVALID = -1,
    DIFFUSE,
    GLOSSY,
    MIRROR,
    REFRACTIVE
};

class PathTracer
{
public:
    PathTracer(int width, int height, int samples, QString lightprobe, bool dof_mode, float focal_l, float aperture);

    void traceScene(QRgb *imageData, const Scene &scene);

private:
    int m_width, m_height, m_samples;
    QString m_probe;
    HDRLoaderResult m_result;
    bool m_success, m_usetex, m_usedof;
    float m_focal_l, m_aperture;

    std::unordered_map<std::string, QImage> m_texmaps;

    Vector3f sampleTexture(Vector2f uvs, const tinyobj::material_t& mat);

    void toneMap(QRgb *imageData, Vector3f *intensityValues);

    int checkType(const tinyobj::material_t *mat);

    Vector3f lightProbe(Vector3f d);

    Vector4f sampleNextDir(tinyobj::real_t ior, Ray ray, Eigen::Vector3f p, Vector3f normal, int *mode); //p=i.hit

    Vector3f getMirrorVec(Vector3f d, Vector3f normal);

    Vector3f getRefractVec(Ray ray, Vector3f p, Vector3f &normal, tinyobj::real_t ior, int *mode); //, float *pdf, int *mode

    Vector3f computeBXDF(int mode, const tinyobj::material_t *mat, Ray *ray, Vector3f normal, Vector3f next_d);

    Vector3f directLighting(const Scene& scene, Vector3f p, Vector3f n, int mode, Vector3f r, float pdf, const tinyobj::material_t *mat);

    Vector3f tracePixel(int x, int y, const Scene &scene, const Eigen::Matrix4f &invViewMatrix);

    Vector3f traceRay(const Ray& r, const Scene &scene, int depth);

    Vector3f barycentricCoords(Vector3f a, Vector3f b, Vector3f c, Vector3f p);

    QRgb getUVcolor(const Mesh *m, int index, QImage *img);

    float haltonSequence(int n, int base);

};

#endif // PATHTRACER_H
