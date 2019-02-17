#ifndef PATHTRACER_H
#define PATHTRACER_H

#include <QImage>

#include "scene/scene.h"

#define N_SAMPLES 100.f       //N number of samples per pixel
#define START_P 0.2f          //initial (stop) probability (for the first 5 bounces)

using namespace Eigen;
using namespace std;

enum MODE {
    INVALID = -1,
    DIFFUSE,
    GLOSSY,
    MIRROR
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

    Vector4f sampleNextDir(const Mesh *m, Vector3f inv_dir, Vector3f normal, int mode);
    Vector3f getMirrorVec(Vector3f inv_dir, Vector3f normal);

    Vector3f tracePixel(int x, int y, const Scene &scene, const Eigen::Matrix4f &invViewMatrix);
    Vector3f traceRay(const Ray& r, const Scene &scene, int depth);
};

#endif // PATHTRACER_H
