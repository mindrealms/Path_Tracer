
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

PathTracer::PathTracer(int width, int height, int samples, QString lightprobe, bool dof_mode, float focal_l, float aperture)
    : m_width(width), m_height(height), m_samples(samples), m_probe(lightprobe),
      m_success(HDRLoader::load(m_probe.toStdString().c_str(), m_result)),
      m_usetex(false), m_usedof(dof_mode), m_focal_l(focal_l), m_aperture(aperture)
{
}

void PathTracer::traceScene(QRgb *imageData, const Scene& scene)
{
    std::clock_t start;
    double duration;
    start = std::clock();

    if (!scene.getTextureMaps().empty()) {
        m_usetex = true;
        m_texmaps = scene.getTextureMaps();
    }

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
    std::cout << "Render time: "<< duration << std::endl;
}


Vector3f PathTracer::tracePixel(int x, int y, const Scene& scene, const Matrix4f &invViewMatrix) {
    Vector3f p(0, 0, 0); //eye
    Vector3f out(0.f, 0.f, 0.f); //accumulate radiance values over N samples

    for (int g_x = 0; g_x < GRID_DIM; g_x++) {
        for (int g_y = 0; g_y < GRID_DIM; g_y++) {

            for (int i = 0; i < m_samples; i++) {

                //pseudorandom point on (x,y) pixel
                float rand_x = static_cast<float>(x) + haltonSequence(g_x + 1.f, BASE_X);
                float rand_y = static_cast<float>(y) + haltonSequence(g_y + 1.f, BASE_Y);

                Vector3f d((2.f * (rand_x) / m_width) - 1.f, 1.f - (2.f * (rand_y) / m_height), -1.f);
                d.normalize();

                if (m_usedof) {

                    Vector3f focal_point = m_focal_l * d;

                    //random point on "disc" (ie. shifting the eye location)
                    float r_x = (2.f * m_aperture * static_cast<float>(rand()) / static_cast<float>(RAND_MAX)) - m_aperture;
                    float r_y = (2.f * m_aperture * static_cast<float>(rand()) / static_cast<float>(RAND_MAX)) - m_aperture;

                    p = Vector3f(r_x, r_y, 0);
                    d = focal_point - p;
                }

                Ray r(p, d);
                r = r.transform(invViewMatrix);
                out += traceRay(r, scene, 0);
            }
        }
    }

    return (out / static_cast<float>(m_samples * GRID_DIM * GRID_DIM)); //average out
}

//p: hit point ; a,b,c: triangle vertices
Vector3f PathTracer::barycentricCoords(Vector3f a, Vector3f b, Vector3f c, Vector3f p) {
    Vector3f pa = p-a, pb = b-p, pc = c-p;
    float area_t = ((b-a).cross(c-a)).norm();
    float a_1 = (pb.cross(pc)).norm() / area_t;
    float a_2 = (pc.cross(pa)).norm() / area_t;
    float a_3 = (pa.cross(pb)).norm() / area_t;

    return Vector3f(a_1, a_2, a_3);
}


QRgb PathTracer::getUVcolor(const Mesh *m, int index, QImage *img) {
    Vector2f uvs = m->getUV(index);
    if (uvs != Vector2f(0.f, 0.f)) {
        return img->pixel(QPoint(uvs[0]*(img->width()), uvs[1]*(img->height())));
    }
    return QRgb(0); //check if this is correct
}

Vector3f PathTracer::traceRay(const Ray& r, const Scene& scene, int depth)
{
    IntersectionInfo i;
    Ray ray(r);
    Vector3f L(0.f, 0.f, 0.f);
    Vector3f tex_color(1.f, 1.f, 1.f); //REMOVE THIS, JUST ADD QCOLOR TO L CONTRIBUTION

    if (scene.getIntersection(ray, &i)) {

        const Mesh * m = static_cast<const Mesh *>(i.object); // mesh intersected
        const Triangle *t = static_cast<const Triangle *>(i.data); // triangle intersected
        const tinyobj::material_t& mat = m->getMaterial(t->getIndex()); // material of triangle
        int mode = checkType(&mat);
        Vector3f normal = t->getNormal(i).normalized();
        Vector4f sample = sampleNextDir(m->getMaterial(t->getIndex()).ior, ray, i.hit, normal, &mode);
        Vector3f next_d = sample.head<3>();

        //texture mapping
        if (m_usetex) {
            QImage img = m_texmaps.at(mat.diffuse_texname);

            int tr_idx = t->getIndex();
            Eigen::Vector3i tr_indices = m->getTriangleIndices(tr_idx);

            //triangle vertex indices and uvs
            int idx1 = tr_indices[0];
            int idx2 = tr_indices[1];
            int idx3 = tr_indices[2];
            Vector2f uv_1 = m->getUV(idx1);
            Vector2f uv_2 = m->getUV(idx2);
            Vector2f uv_3 = m->getUV(idx3);

            Vector3f barys = barycentricCoords(m->getVertex(idx1), m->getVertex(idx2), m->getVertex(idx3), i.hit);

            //uv corresponding to hit point
            Vector2f uv_hit = uv_1 * barys[0] + uv_2 * barys[1] + uv_3 * barys[2];
            QRgb qcol = img.pixel(QPoint(uv_hit[0]*(img.width()), img.height() - uv_hit[1]*(img.height())));
            tex_color = Vector3f(qRed(qcol)/255.f, qGreen(qcol)/255.f, qBlue(qcol)/255.f);

            L += tex_color;
        }

        //direct lighting computation                                 //sample[3] = pdf
        if (mode != MIRROR && mode != REFRACTIVE) {
            L += Vector3f(directLighting(scene, i.hit, normal, mode, ray.d, sample[3], &mat).array());
        }

        //bxdf computation
        Vector3f bxdf = computeBXDF(mode, &mat, &ray, normal, next_d);

        float pdf_rr; //russian roulette - continue probability
        switch(static_cast<int>(depth < 5)) {
        case 0: //bounce past 5th: continue p weighed according to bxdf
            pdf_rr = bxdf.norm();
            break;
        default: //first 5 (non-mirror) bounces, 80% continue prob
            pdf_rr = START_P;
            break;
        }

        //indirect lighting
        if (static_cast<float>(rand())/RAND_MAX < pdf_rr) {
            Ray new_dir(i.hit, next_d);
            float dot = (new_dir.d).dot(normal); //already normalized
            float denom = pdf_rr * sample[3]; // --------->>>>>>>>>> (1/(2*M_PI)) ?? sample[3]

            Vector3f radiance;
            if (mode == MIRROR || mode == REFRACTIVE) {
                radiance = traceRay(new_dir, scene, 0); // STUPID ASS BUG YOU RIPPED MY SOUL OUT BUT I FOUND YA
            } else  {
                radiance = traceRay(new_dir, scene, depth + 1);
            }

            L += (Vector3f(radiance.array() * bxdf.array()) * dot) / denom;
        }
        if (depth == 0) { //surface is a luminaire
            L += Vector3f(mat.emission[0], mat.emission[1], mat.emission[2]);
        }

    //no intersection, check if we have a light probe
    } else if (m_success) {

        //count probe contribution to be able to see background environment
        if (depth == 0) {
            L += lightProbe(ray.d);
        }
    }

    return L;
}


Vector3f PathTracer::lightProbe(Vector3f d) {

    if (m_success) {

        float r = (1.f/M_PI) * acos(d[2]) / sqrt(d[0]*d[0] + d[1]*d[1]);
        Vector2f uv = Vector2f((d[0] * r + 1.f)/2.f, 1.f - (d[1] * r + 1.f)/2.f); //hdr image coordinate in [0,1] with origin top left

        //"resizing" uv according to img dimensions
        int col = floor(uv[0] * m_result.width);
        int row = floor(uv[1] * m_result.height);

        int index = ((row * m_result.width) + col) * 3; //3 floats per pixel

        return Vector3f(m_result.cols[index], m_result.cols[index+1], m_result.cols[index+2]);
    }
    return Vector3f(0.f, 0.f, 0.f);
}


Vector3f PathTracer::computeBXDF(int mode, const tinyobj::material_t *mat, Ray *ray, Vector3f normal, Vector3f next_d) {
    switch(mode) {
    case DIFFUSE: {
        return Vector3f(mat->diffuse[0]/M_PI, mat->diffuse[1]/M_PI, mat->diffuse[2]/M_PI);
    }
    case GLOSSY: {
        float k = ((mat->shininess + 2) / (2*M_PI)) * pow(getMirrorVec(ray->d, normal).dot(next_d), mat->shininess);
        return Vector3f(mat->specular[0]*k, mat->specular[1]*k, mat->specular[2]*k);
    }
    case MIRROR:
    case REFRACTIVE: {
        float k = 1.f / next_d.dot(normal);
        return Vector3f(k, k, k); //Serious note: KKK completely unintentional.
    }
    default: {
        return Vector3f(0.f, 0.f, 0.f);
    }
    }
}


Vector3f PathTracer::directLighting(const Scene& scene, Vector3f p, Vector3f n, int mode, Vector3f r, float pdf, const tinyobj::material_t *mat) {

    std::vector<PathLight> lights = scene.getPathLights();

    //scene contains emissive materials (light objects)
    if (lights.size() != 0) {

        int index = rand() % lights.size(); //random light index

        //select triangles weighted by area
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

            const Mesh * m = static_cast<const Mesh *>(i.object); // mesh intersected
            const Triangle *t = static_cast<const Triangle *>(i.data); // triangle intersected
            const tinyobj::material_t& material = m->getMaterial(t->getIndex()); // material of triangle

            int type = checkType(&material);
            if (type == REFRACTIVE || type == MIRROR) {
                return Vector3f(0.f, 0.f, 0.f);
            }

            //surface is the luminaire
            if ((i.hit[0] - tri_p[0] < EPSILON) && (i.hit[1] - tri_p[1] < EPSILON) && (i.hit[2] - tri_p[2] < EPSILON)) {
                Ray from_light(tri_p, (-dir).normalized());
                Vector3f bxdf = computeBXDF(mode, mat, &from_light, n, -r);

                float o_dot = min(1.f, max(0.f, (dir.normalized()).dot(n))); //object surface dot
                float l_dot = min(1.f, max(0.f, ((-dir).normalized()).dot(t->getNormal(i).normalized()))); //light surface dot

                return Vector3f((lights[index].emission).array() * bxdf.array() *
                        o_dot * l_dot * lights[index].area) /  (dir.norm() * dir.norm() * pdf);
            }
        }

    } else if (m_success) { //light probe is light source

        Vector4f sample = sampleNextDir(mat->ior, Ray(Vector3f(0.f,0.f,0.f), r), Vector3f(0.f,0.f,0.f), n, &mode);
        Vector3f next_d = sample.head<3>();

        IntersectionInfo i;
        Ray to_void(p, next_d.normalized());
        if (!(scene.getIntersection(to_void, &i))) {
            Ray from_void(to_void.d, -next_d.normalized());
            Vector3f bxdf = computeBXDF(mode, mat, &from_void, n, -r);
            float o_dot = min(1.f, max(0.f, (next_d.normalized()).dot(n)));

            return Vector3f(lightProbe(next_d).array() * bxdf.array() *
                    o_dot) /  (next_d.norm() * next_d.norm() * pdf);
        }
    }

    return Vector3f(0.f, 0.f, 0.f);
}


/* returns Vector4f, where .xyz = outoing direction vector w_o, and .w = pdf float value */
Vector4f PathTracer::sampleNextDir(tinyobj::real_t ior, Ray ray, Vector3f p, Vector3f normal, int *mode) {

    Vector3f w_o(0.f, 0.f, 0.f);
    float phi, pdf;

    switch(*mode){
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
        w_o = getRefractVec(ray, p, normal, ior, mode).normalized();
        break; }
    }

    return vec3Tovec4(w_o, pdf);
}


Vector3f PathTracer::getMirrorVec(Vector3f d, Vector3f normal) {
    return (2.f * d.dot(normal) * normal - d).normalized();
}

Vector3f PathTracer::getRefractVec(Ray ray, Vector3f p, Vector3f &normal, tinyobj::real_t ior, int *mode) { //, float *pdf, int *mode

    float cos_i = max(-1.f, min(1.f, ray.d.dot(normal)));
    float n_i = 1.f, n_t = ior;

    if (cos_i < 0) { //out -> in
        cos_i = -cos_i;
    } else { //in -> out
        swap(n_i, n_t);
        normal = -normal;
    }

    float r_o = pow((n_i - n_t) / (n_i + n_t), 2.f);
    float schlick = r_o + (1.f - r_o) * pow((1.f - cos_i), 5.f); //% reflected
    float k = 1.f - ((n_i/n_t)*(n_i/n_t)) * (1.f - cos_i*cos_i);

    if (k < 0.f) {
        *mode = MIRROR;
        return getMirrorVec(ray.d, -normal); //total internal reflection
    } else if (static_cast<float>(rand())/RAND_MAX < schlick) {
        *mode = MIRROR;
        return getMirrorVec(-ray.d, normal); //ideal reflection
    }

    //set up attenuation (only when the ray goes inside the new medium)
    float distance = 10.f * (p - ray.o).norm(); //mult by 10 bc units are too small so virtual distance is < 1 unit
    float attenuation = (1.f / min(1.f, (distance)));

    return (((n_i/n_t) * ray.d + ((n_i/n_t)*cos_i - sqrt(k)) * normal)) * attenuation; //refraction (snell)
}


int PathTracer::checkType(const tinyobj::material_t *mat) {
    switch(mat->illum) {
    case 2: {
        float diff = Vector3f(mat->diffuse[0], mat->diffuse[1], mat->diffuse[2]).norm();
        float spec = Vector3f(mat->specular[0], mat->specular[1], mat->specular[2]).norm();
        float type = std::max(diff,spec); // for now ! ! !

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

            //L divided by the average radiance across the scene, or the log -- helps to normalize to the overall brightness
        }
    }
}

//low discrepancy sampling
float PathTracer::haltonSequence(int index, int base){
  float f = 1.f, random = 0.f;
  while (index > 0){
    f = f/base;
    random += f * (index % base);
    index = index/base;
  }

  return random;
}
