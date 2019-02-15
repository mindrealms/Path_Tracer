#include "pathtracer.h"

#include <iostream>

#include <Eigen/Dense>

#include <util/CS123Common.h>

#include <ctime>



/* Example code for accessing materials provided by a .mtl file
    const Mesh * m = static_cast<const Mesh *>(i.object); //Get the mesh that was intersected
    const Triangle *t = static_cast<const Triangle *>(i.data); //Get the triangle in the mesh that was intersected
    const tinyobj::material_t& mat = m->getMaterial(t->getIndex()); //Get the material of the triangle from the mesh
    const tinyobj::real_t *d = mat.diffuse; //Diffuse color as array of floats
    const std::string diffuseTex = mat.diffuse_texname; //Diffuse texture name
*/

PathTracer::PathTracer(int width, int height)
    : m_width(width), m_height(height)
{
}

void PathTracer::traceScene(QRgb *imageData, const Scene& scene)
{
    std::clock_t start;
    double duration;
    start = std::clock();

    Vector3f intensityValues[m_width * m_height];
    Matrix4f invViewMat = (scene.getCamera().getScaleMatrix() * scene.getCamera().getViewMatrix()).inverse();
    for(int y = 0; y < m_height; ++y) {
//        #pragma omp parallel for
        for(int x = 0; x < m_width; ++x) {
            std::cout << "pixel " << x << std::endl;
            int offset = x + (y * m_width);
            intensityValues[offset] = tracePixel(x, y, scene, invViewMat);
        }
    }

    toneMap(imageData, intensityValues);

    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    std::cout<<"Render time: "<< duration <<'\n';
}

__attribute__((force_align_arg_pointer)) Vector3f PathTracer::tracePixel(int x, int y, const Scene& scene, const Matrix4f &invViewMatrix)
{
    Vector3f p(0, 0, 0); //eye
    Vector3f out(0.f, 0.f, 0.f); //accumulate radiance values over N samples

    for (int i = 0; i < N_SAMPLES; i++) {

        //random point on (x,y) pixel
        float rand_x = static_cast<float>(x) + (static_cast<float>(rand()) / RAND_MAX) + 1;
        float rand_y = static_cast<float>(y) + (static_cast<float>(rand()) / RAND_MAX) + 1;

        Vector3f d((2.f * (rand_x) / m_width) - 1.f, 1.f - (2.f * (rand_y) / m_height), -1);
        d.normalize();

        Ray r(p, d);
        r = r.transform(invViewMatrix);
        out += traceRay(r, scene, 0); //traces ray through current pixel, depth = 0
    }
    return (out/static_cast<float>(N_SAMPLES)); //average out
}

Vector3f PathTracer::traceRay(const Ray& r, const Scene& scene, int depth)
{
    IntersectionInfo i;
    Ray ray(r);
    Vector3f L(0.f, 0.f, 0.f);

    if(scene.getIntersection(ray, &i)) {

        const Mesh * m = static_cast<const Mesh *>(i.object); //Get the mesh that was intersected
        const Triangle *t = static_cast<const Triangle *>(i.data); //Get the triangle in the mesh that was intersected
        const tinyobj::material_t& mat = m->getMaterial(t->getIndex()); //Get the material of the triangle from the mesh

        //surface is a light source
        if (mat.emission[0] || mat.emission[1] || mat.emission[2]) {
            L = Vector3f(mat.emission[0], mat.emission[1], mat.emission[2]); //emitted
        }

        int mode = checkType(&mat);
        Vector3f normal = t->getNormal(i).normalized();
        Vector4f sample = sampleNextDir(m, ray.inv_d, normal, mode);
        Vector3f next_d = sample.head<3>();

        //brdf computation
        Vector3f brdf;
        switch(mode) {
        case DIFFUSE: {
            brdf = Vector3f(mat.diffuse[0]/M_PI, mat.diffuse[1]/M_PI, mat.diffuse[2]/M_PI);
            break;
        }
        case GLOSSY: {
            float k = (mat.shininess + 2) / (2*M_PI) * pow(getMirrorVec(m->inverseTransform * ray.inv_d, normal).dot(next_d), mat.shininess);
            brdf = Vector3f(mat.specular[0]*k, mat.specular[1]*k, mat.specular[2]*k);
            break;
        }
        case MIRROR: {
            float k = next_d.dot(normal);
            brdf = Vector3f(k, k, k);
            break;
        }
        }

        float pdf_rr = brdf.norm(); //russian roulette - continue probability

        switch(static_cast<int>(depth < 5)) {
        case 0:
            if (static_cast<float>(rand())/RAND_MAX < pdf_rr) {
                break;
            }
        default: //first 5 bounces, 80% continue prob
            if (START_P < pdf_rr) {
                Ray new_dir(m->inverseTransform * i.hit, next_d);
                new_dir.transform(m->transform);

                Vector3f radiance = traceRay(new_dir, scene, depth + 1); //recursion

                float dot = (m->transform * new_dir.d).dot(normal);
                float denom = sample[3] * pdf_rr;

                //change to Vector3f(r.array() * b.array()) ....
                L[0] += ( radiance[0] * brdf[0] * dot ) / denom; //red
                L[1] += ( radiance[1] * brdf[1] * dot ) / denom; //green
                L[2] += ( radiance[2] * brdf[2] * dot ) / denom; //blue
            }
            break;
        }
        if (depth == 0) {
            L += Vector3f(mat.emission[0], mat.emission[1], mat.emission[2]); //emitted
        }

//        if (static_cast<float>(rand())/RAND_MAX < pdf_rr) {
//            Ray new_dir(m->inverseTransform * i.hit, next_d);
//            new_dir.transform(m->transform);

//            Vector3f radiance = traceRay(new_dir, scene, depth + 1); //recursion

//            float dot = (m->transform * new_dir.d).dot(normal);
//            float denom = sample[3] * pdf_rr;

//            //change to Vector3f(r.array() * b.array()) ....
//            L[0] += ( radiance[0] * brdf[0] * dot ) / denom; //red
//            L[1] += ( radiance[1] * brdf[1] * dot ) / denom; //green
//            L[2] += ( radiance[2] * brdf[2] * dot ) / denom; //blue
//        }
//        if (depth == 0) {
//            L += Vector3f(mat.emission[0], mat.emission[1], mat.emission[2]); //emitted
//        }
    }
    return L;
}

/* returns Vector4f, where .xyz = outoing direction vector w_o, and .w = pdf float value */
Vector4f PathTracer::sampleNextDir(const Mesh *m, Vector3f inv_dir, Vector3f normal, int mode) {

    Vector3f w_o(0.f, 0.f, 0.f);
    float phi, pdf = 0.f;

    if ((mode == DIFFUSE) || (mode == GLOSSY)) {
        //random xi (ξ) numbers for hemisphere sampling
        float xi_1 = static_cast<float>(rand())/RAND_MAX, xi_2 = static_cast<float>(rand())/RAND_MAX;

        //defining pdf, and random phi/theta angles
        pdf = 1.f/(2*M_PI);
        phi = 2.f*M_PI * xi_1;
        float theta = acos(1.f - xi_2);

        //vector normal to the plane formed by incident wi and surface normal
        Vector3f orth_v = ((m->inverseTransform * inv_dir).normalized()).cross(normal);

        //transformation matrix rotating vector by theta rad around orth_v
        Transform<float,3,Affine> theta_rot = Transform<float,3,Affine>::Identity();
        theta_rot.rotate(AngleAxisf(theta, orth_v));
        w_o = theta_rot * normal;

        //transformation matrix rotation vector by phi rad around surface normal
        Transform <float,3,Affine> phi_rot = Transform<float,3,Affine>::Identity();
        phi_rot.rotate(AngleAxisf(phi, normal));
        w_o = (phi_rot * w_o).normalized(); //random outgoing direction yay!

    } else if (mode == MIRROR) {
        pdf = 1.f;
        w_o = getMirrorVec(m->inverseTransform * inv_dir, normal);
    }

    return vec3Tovec4(w_o, pdf);
}


Vector3f PathTracer::getMirrorVec(Vector3f inv_dir, Vector3f normal) {
    Transform <float,3,Affine> refl_rot = Transform<float,3,Affine>::Identity();
    refl_rot.rotate(AngleAxisf(M_PI, normal));
    return (refl_rot * (inv_dir)).normalized();
}


int PathTracer::checkType(const tinyobj::material_t *mat) {
    switch(mat->illum) {
    case 2: {
        float diff = Vector3f(mat->diffuse[0], mat->diffuse[1], mat->diffuse[2]).norm();
        float spec = Vector3f(mat->specular[0], mat->specular[1], mat->specular[2]).norm();
        float type = std::max(diff,spec); // for now

        if (type == diff) { return DIFFUSE; }
        else if (type == spec) { return GLOSSY; }
        else { return INVALID; }
    }
    case 5: {
        return MIRROR;
    }
    default: {
        return INVALID; //woops
    }
    }
}

void PathTracer::toneMap(QRgb *imageData, Vector3f *intensityValues) {
    //iterate through img pixels
    for(int y = 0; y < m_height; ++y) {
        for(int x = 0; x < m_width; ++x) {
            int offset = x + (y * m_width);

            //reinhard operator [0,1] * 255
            float red = (intensityValues[offset][0] / (1+intensityValues[offset][0]))*255.f;
            float green = (intensityValues[offset][1] / (1+intensityValues[offset][1]))*255.f;
            float blue = (intensityValues[offset][2] / (1+intensityValues[offset][2]))*255.f;
            imageData[offset] = qRgb(red, green, blue);

        }
    }

}
