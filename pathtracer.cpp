
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
            int offset = x + (y * m_width);
            intensityValues[offset] = tracePixel(x, y, scene, invViewMat);
        }
    }

    toneMap(imageData, intensityValues);

    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    std::cout<<"Render time: "<< duration << std::endl;
}

Vector3f PathTracer::tracePixel(int x, int y, const Scene& scene, const Matrix4f &invViewMatrix)
{
    Vector3f p(0, 0, 0); //eye
    Vector3f out(0.f, 0.f, 0.f); //accumulate radiance values over N samples

    for (int i = 0; i < N_SAMPLES; i++) {

        //random point on (x,y) pixel
        float rand_x = static_cast<float>(x) + (static_cast<float>(rand()) / RAND_MAX); //+ 1.f
        float rand_y = static_cast<float>(y) + (static_cast<float>(rand()) / RAND_MAX); // + 1.f

        Vector3f d((2.f * (rand_x) / m_width) - 1.f, 1.f - (2.f * (rand_y) / m_height), -1.f);
        d.normalize();

        Ray r(p, d);
        r = r.transform(invViewMatrix);
        out += traceRay(r, scene, 0);
    }
    return (out/N_SAMPLES); //average out
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

        int mode = checkType(&mat);
        Vector3f normal = t->getNormal(i).normalized();
        Vector4f sample = sampleNextDir(m->getMaterial(t->getIndex()).ior, ray, normal, mode);
        Vector3f next_d = sample.head<3>();

        if (mode != MIRROR) {
            L = Vector3f(directLighting(scene, i.hit, normal).array() * Vector3f(mat.ambient[0], mat.ambient[1], mat.ambient[2]).array()); //w.sp
        }

        //bsdf computation
        Vector3f bsdf;
        switch(mode) {
        case DIFFUSE: {
            bsdf = Vector3f(mat.diffuse[0]/M_PI, mat.diffuse[1]/M_PI, mat.diffuse[2]/M_PI);
            break;
        }
        case GLOSSY: {
            float k = ((mat.shininess + 2) / (2*M_PI)) * pow(getMirrorVec(ray.d, normal).dot(next_d), mat.shininess);
            bsdf = Vector3f(mat.specular[0]*k, mat.specular[1]*k, mat.specular[2]*k);
            break;
        }
        case MIRROR:
//        {
//            float k = 1.f / next_d.dot(normal);
//            bsdf = Vector3f(k, k, k); //Disclaimer: KKK completely unintentional.
//            break;
//        }
        case REFRACTIVE: {
            float k = 1.f / next_d.dot(normal);
            bsdf = Vector3f(k, k, k); //Disclaimer: KKK completely unintentional.
            break;
        }
        }

        float pdf_rr; //russian roulette - continue probability
        switch(static_cast<int>(depth < 5)) {
        case 0: //bounce past 5th: continue p weighed according to bsdf but clamped above 0.7 (else it was speckly)
            pdf_rr = max(CLAMP_P, bsdf.norm());
            break;
        default: //first 5 (non-mirror) bounces, 80% continue prob
            pdf_rr = START_P;
            break;
        }

        if (static_cast<float>(rand())/RAND_MAX < pdf_rr) {
            Ray new_dir(i.hit, next_d); //o.sp
            float dot = (new_dir.d).dot(normal);
            float denom = sample[3] * pdf_rr;
            new_dir.transform(m->transform); //w.sp

            Vector3f radiance;
            if (mode == MIRROR) {
                radiance = traceRay(new_dir, scene, depth); //not increasing depth for mirror rays
            } else  {
                radiance = traceRay(new_dir, scene, depth + 1);
            }

            L += (Vector3f(radiance.array() * bsdf.array()) * dot) / denom;
        }
        if (depth == 0) { //surface is a luminaire
            L += Vector3f(mat.emission[0], mat.emission[1], mat.emission[2]);
        }
    }
    return L;
}


Vector3f PathTracer::directLighting(const Scene& scene, Vector3f p, Vector3f n) {

    std::vector<PathLight> lights = scene.getPathLights();

    int index = rand() % lights.size(); //random light index
    int r_face = rand() % (lights[index].n_triangles); //random face/triangle
    std::vector<Vector3f> face = lights[index].faces[r_face]; //1 face (3 points)

    //random point on triangle
    float r_1 = static_cast<float>(rand())/RAND_MAX;
    float r_2 = static_cast<float>(rand())/RAND_MAX;
    Vector3f tri_p = (1.f - sqrt(r_1)) * face[0] + (sqrt(r_1) * (1.f - r_2)) * face[1] + (sqrt(r_1) * r_2) * face[2];
    Vector3f dir = tri_p - p;

    IntersectionInfo i;
    Ray to_light(p, dir.normalized());

    if (scene.getIntersection(to_light, &i)) {
        if ((i.hit[0] - tri_p[0] < EPSILON) && (i.hit[1] - tri_p[1] < EPSILON) && (i.hit[2] - tri_p[2] < EPSILON)) {

            float o_dot = (dir.normalized()).dot(n); //object surface dot
            const Triangle *t = static_cast<const Triangle *>(i.data);
            float l_dot = ((-dir).normalized()).dot(t->getNormal(i).normalized()); //light surface dot

            //clampin' -- but not clampin' -- just exitin'
            if (o_dot < 0 || l_dot < 0) {
                return Vector3f(0.f, 0.f, 0.f);
            }

            return ((lights[index].emission) * ((o_dot * l_dot) / (dir.norm() * dir.norm()))) * lights[index].area;
        }
    }

    return Vector3f(0.f, 0.f, 0.f);
}


/* returns Vector4f, where .xyz = outoing direction vector w_o, and .w = pdf float value */
Vector4f PathTracer::sampleNextDir(tinyobj::real_t ior, Ray ray, Vector3f normal, int mode) {

    Vector3f w_o(0.f, 0.f, 0.f);
    float phi, pdf = 0.f;

    switch(mode){
    case DIFFUSE:
    case GLOSSY: {
        //random xi (ξ) numbers for hemisphere sampling
        float xi_1 = static_cast<float>(rand())/RAND_MAX, xi_2 = static_cast<float>(rand())/RAND_MAX;

        //defining pdf, and random phi/theta angles
        pdf = 1.f/(2*M_PI);
        phi = 2.f*M_PI * xi_1;
        float theta = acos(xi_2);

        //vector normal to the plane formed by incident wi and surface normal
        Vector3f orth_v = (ray.inv_d).cross(normal);

        //transformation matrix rotating vector by theta rad around orth_v
        Transform<float,3,Affine> theta_rot = Transform<float,3,Affine>::Identity();
        theta_rot.rotate(AngleAxisf(theta, orth_v));
        w_o = theta_rot * normal;

        //transformation matrix rotation vector by phi rad around surface normal
        Transform <float,3,Affine> phi_rot = Transform<float,3,Affine>::Identity();
        phi_rot.rotate(AngleAxisf(phi, normal));
        w_o = (phi_rot * w_o).normalized(); //random outgoing direction yay!
        break; }
    case MIRROR: {
        pdf = 1.f;
        w_o = -getMirrorVec(ray.d, normal);
        break; }
    case REFRACTIVE: {
        pdf = 1.f;
        w_o = -getRefractVec(&ray, normal, ior);
        break; }
    }

//    if ((mode == DIFFUSE) || (mode == GLOSSY)) {
//        //random xi (ξ) numbers for hemisphere sampling
//        float xi_1 = static_cast<float>(rand())/RAND_MAX, xi_2 = static_cast<float>(rand())/RAND_MAX;

//        //defining pdf, and random phi/theta angles
//        pdf = 1.f/(2*M_PI);
//        phi = 2.f*M_PI * xi_1;
//        float theta = acos(xi_2);

//        //vector normal to the plane formed by incident wi and surface normal
//        Vector3f orth_v = (ray.inv_d).cross(normal);

//        //transformation matrix rotating vector by theta rad around orth_v
//        Transform<float,3,Affine> theta_rot = Transform<float,3,Affine>::Identity();
//        theta_rot.rotate(AngleAxisf(theta, orth_v));
//        w_o = theta_rot * normal;

//        //transformation matrix rotation vector by phi rad around surface normal
//        Transform <float,3,Affine> phi_rot = Transform<float,3,Affine>::Identity();
//        phi_rot.rotate(AngleAxisf(phi, normal));
//        w_o = (phi_rot * w_o).normalized(); //random outgoing direction yay!

//    } else if (mode == MIRROR) {
//        pdf = 1.f;
//        w_o = -getMirrorVec(ray.d, normal);

//    } else if (mode == REFRACTIVE) {
//        pdf = 1.f;
//        w_o = getRefractVec(ray.d, normal, ior);
//    }

    return vec3Tovec4(w_o, pdf);
}


Vector3f PathTracer::getMirrorVec(Vector3f d, Vector3f normal) {
    return (2.f * d.dot(normal) * normal - d).normalized();
}

Vector3f PathTracer::getRefractVec(Ray *ray, Vector3f normal, tinyobj::real_t ior) {

    float cos_i = min(1.f, max(-1.f, (ray->d).dot(normal)));
    float etai = 1.f, etat = ior;
//    float index = 1.f/ior;

    if (cos_i < 0) { //out -> in
        cos_i = -cos_i;
//        ior = 1.f / ior;
//        index = 1.f/ior;
    } else { //in -> out
//        index = ior;
        std::swap(etai, etat);
        normal = -normal;
    }

//    std::cout << "cos in = " << cos_i << std::endl;
    float eta = etai/etat;
    float k = (1.f - (eta*eta) * (1.f - cos_i*cos_i));
//    std::cout << "root = " << root << std::endl;
//    std::cout << "cos t = " << cos_t << std::endl;
    return (k < 0.f ? Vector3f(0.f, 0.f, 0.f) : (eta * ray->d + (eta*cos_i - sqrt(k)) * normal));
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
    case 7: {
        return REFRACTIVE;
    }
    default: {
        return INVALID; //woopsay
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

