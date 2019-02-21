
#ifndef PATHTRACER_H
#define PATHTRACER_H

#include <QImage>

#include "scene/scene.h"

#define N_SAMPLES 50.f      //N number of samples per pixel
#define START_P 0.8f        //initial (stop) probability (for the first 5 bounces)
#define CLAMP_P 0.7f        //stop probability for > 5 bounces (edit later maybe???? is it too high?)
#define EPSILON 0.01f       //epsilon term (direct lighting -- to check for light intersection vs occlusion)

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
    PathTracer(int width, int height);

    void traceScene(QRgb *imageData, const Scene &scene);

private:
    int m_width, m_height;

    void toneMap(QRgb *imageData, Vector3f *intensityValues);

    int checkType(const tinyobj::material_t *mat);

    Vector4f sampleNextDir(tinyobj::real_t ior, Ray ray, Vector3f normal, int mode);
    Vector3f getMirrorVec(Vector3f d, Vector3f normal);
    Vector3f getRefractVec(Ray *ray, Vector3f normal, tinyobj::real_t ior);

    Vector3f directLighting(const Scene& scene, Vector3f p, Vector3f n);

    Vector3f tracePixel(int x, int y, const Scene &scene, const Eigen::Matrix4f &invViewMatrix);
    Vector3f traceRay(const Ray& r, const Scene &scene, int depth);
};

#endif // PATHTRACER_H
