#include "scene.h"

#include "shape/Sphere.h"

#include <util/CS123XmlSceneParser.h>

#include <util/CS123Common.h>

#include <Eigen/Geometry>

#include <iostream>

#include <Eigen/StdVector>

#define TINYOBJLOADER_IMPLEMENTATION
#include "util/tiny_obj_loader.h"

using namespace Eigen;

static std::vector<PathLight> _lights = std::vector<PathLight>();
static std::unordered_map<std::string, QImage> _texmaps = std::unordered_map<std::string, QImage>();

Scene::Scene()
{
}

Scene::~Scene()
{
    for(unsigned int i = 0; i < _objects->size(); ++i) {
        Object * o = (*_objects)[i];
        delete o;
    }
    delete _objects;
    delete m_bvh;
}

bool Scene::load(QString filename, Scene **scenePointer)
{
    CS123XmlSceneParser parser(filename.toStdString());
    if(!parser.parse()) {
        return false;
    }
    CS123SceneCameraData cameraData;
    parser.getCameraData(cameraData);
    BasicCamera camera(cameraData.pos.head<3>(),
                       cameraData.look.head<3>(),
                       cameraData.up.head<3>(),
                       cameraData.heightAngle,
                       (float)IMAGE_WIDTH / (float)IMAGE_HEIGHT);
    Scene *scene = new Scene;
    scene->setCamera(camera);

    CS123SceneGlobalData globalData;
    parser.getGlobalData(globalData);
    scene->setGlobalData(globalData);

    CS123SceneLightData lightData;
    for(int i = 0, size = parser.getNumLights(); i < size; ++i) {
        parser.getLightData(i, lightData);
        scene->addLight(lightData);
    }

    QFileInfo info(filename);
    QString dir = info.path();
    CS123SceneNode *root = parser.getRootNode();
    if(!parseTree(root, scene, dir.toStdString() + "/")) {
        return false;
    }

    *scenePointer = scene;

    return true;
}

void Scene::setBVH(const BVH &bvh)
{
    m_bvh = new BVH(bvh);
}

bool Scene::parseTree(CS123SceneNode *root, Scene *scene, const std::string &baseDir)
{
    std::vector<Object *> *objects = new std::vector<Object *>;
    parseNode(root, Affine3f::Identity(), objects, baseDir);
    if(objects->size() == 0) {
        return false;
    }
    std::cout << "Parsed tree, creating BVH" << std::endl;
    BVH *bvh = new BVH(objects);

    scene->_objects = objects;
    scene->setBVH(*bvh);

    return true;
}

void Scene::parseNode(CS123SceneNode *node, const Affine3f &parentTransform, std::vector<Object *> *objects, const std::string &baseDir)
{
    Affine3f transform = parentTransform;
    for(CS123SceneTransformation *trans : node->transformations) {
        switch(trans->type) {
        case TRANSFORMATION_TRANSLATE:
            transform = transform * Translation<float, 3>(trans->translate);
            break;
        case TRANSFORMATION_SCALE:
            transform = transform * Scaling(trans->scale);
            break;
        case TRANSFORMATION_ROTATE:
            transform = transform * AngleAxis<float>(trans->angle, trans->rotate);
            break;
        case TRANSFORMATION_MATRIX:
            transform = transform * trans->matrix;
            break;
        }
    }
    for(CS123ScenePrimitive *prim : node->primitives) {
        addPrimitive(prim, transform, objects, baseDir);
    }
    for(CS123SceneNode *child : node->children) {
        parseNode(child, transform, objects, baseDir);
    }
}

void Scene::addPrimitive(CS123ScenePrimitive *prim, const Affine3f &transform, std::vector<Object *> *objects, const std::string &baseDir)
{
    switch(prim->type) {
    case PrimitiveType::PRIMITIVE_MESH: {
        std::cout << "Loading mesh " << prim->meshfile << std::endl;
        objects->push_back(loadMesh(prim->meshfile, transform, baseDir));
        std::cout << "Done loading mesh" << std::endl;
        break; }
    default:
        std::cerr << "We don't handle any other formats yet" << std::endl;
        break;
    }
}


//** everything is stored in giant vectors, so shapes/mesh vertex indices/vertex attributes are all parallel-stored */
Mesh *Scene::loadMesh(std::string filePath, const Affine3f &transform, const std::string &baseDir)
{
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;


    QFileInfo info(QString((baseDir + filePath).c_str()));
    std::string err;
    bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &err,
                                info.absoluteFilePath().toStdString().c_str(), (info.absolutePath().toStdString() + "/").c_str(), true);
    if(!err.empty()) {
        std::cerr << err << std::endl;
    }

    if(!ret) {
        std::cerr << "Failed to load/parse .obj file" << std::endl;
        return nullptr;
    }

    std::vector<Vector3f> vertices;
    std::vector<Vector3f> normals;
    std::vector<Vector3f> colors;
    std::vector<Vector2f> uvs;
    std::vector<int> materialIds;
    std::vector<Vector3i> faces;

    std::vector<std::string> texmapIds;

    //TODO populate vectors and use tranform
    for (size_t s = 0; s < shapes.size(); s++) { //iterate through all shapes
        size_t index_offset = 0;
        for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) { //iterate through shape mesh's face vertices
            unsigned int fv = shapes[s].mesh.num_face_vertices[f];

            Vector3i face;
            for (size_t v = 0; v < fv; v++) { //iterate through curr face's vertices
                tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
                tinyobj::real_t vx = attrib.vertices[3*idx.vertex_index+0];
                tinyobj::real_t vy = attrib.vertices[3*idx.vertex_index+1];
                tinyobj::real_t vz = attrib.vertices[3*idx.vertex_index+2];
                tinyobj::real_t nx;
                tinyobj::real_t ny;
                tinyobj::real_t nz;
                tinyobj::real_t tx;
                tinyobj::real_t ty;

                if (idx.normal_index != -1) {
                    nx = attrib.normals[3*idx.normal_index+0];
                    ny = attrib.normals[3*idx.normal_index+1];
                    nz = attrib.normals[3*idx.normal_index+2];
                } else { //if not defined in file
                    nx = 0;
                    ny = 0;
                    nz = 0;
                }
                if (idx.texcoord_index != -1) {
                    tx = attrib.texcoords[2*idx.texcoord_index+0];
                    ty = attrib.texcoords[2*idx.texcoord_index+1];
                } else { //if not defined in file
                    tx = 0;
                    ty = 0;
                }

                tinyobj::real_t red = attrib.colors[3*idx.vertex_index+0];
                tinyobj::real_t green = attrib.colors[3*idx.vertex_index+1];
                tinyobj::real_t blue = attrib.colors[3*idx.vertex_index+2];

                face[v] = vertices.size();
                vertices.push_back(transform * Vector3f(vx, vy, vz));
                normals.push_back((transform.linear() * Vector3f(nx, ny, nz)).normalized());
                uvs.push_back(Vector2f(tx, ty));
                colors.push_back(Vector3f(red, green, blue));
            }

            int mat_id = shapes[s].mesh.material_ids[f];

            faces.push_back(face);
            materialIds.push_back(mat_id);

            //loading texture maps in _texmaps set
            std::string img_name = materials[mat_id].diffuse_texname;
            if (_texmaps.find(img_name) == _texmaps.end() && img_name != "") {
//                std::cout << "name = " << img_name << std::endl;
                QImage image = QImage(materials[mat_id].diffuse_texname.data());
                _texmaps[img_name] = image;
            }

            //add emissive triangles to _lights vector
            Vector3f emitted = Vector3f(materials[mat_id].emission[0],
                                        materials[mat_id].emission[1],
                                        materials[mat_id].emission[2]);

            if (emitted != Vector3f(0.f, 0.f, 0.f)) { //material is emissive
                bool not_added = 0;
                if (_lights.size() != 0) { //if not empty, check if already stored
                    for (unsigned int i=0; i < _lights.size(); i++) {
                        if (_lights[i].id == mat_id) {
                            std::vector<Vector3f> triangle = {vertices[face[0]], vertices[face[1]], vertices[face[2]]};
                            _lights[i].faces.push_back(triangle);
                            _lights[i].n_triangles += 1;
                            assert(_lights[i].emission == emitted);
                            assert(_lights[i].intensity == emitted.norm());
                        } else {
                            not_added = 1;
                        }
                    }
                } else if ((_lights.size() == 0) || not_added ){ //if empty, or not already stored
                    std::vector<Vector3f> triangle = {vertices[face[0]], vertices[face[1]], vertices[face[2]]};

                    PathLight new_light;
                    new_light.emission = emitted;
                    new_light.intensity = emitted.norm(); //assumes uniform emission over all light tris
                    new_light.n_triangles += 1;
                    new_light.id = mat_id;
                    new_light.faces.push_back(triangle);
                    _lights.push_back(new_light);
                }
            }

            index_offset += fv;
        }
    }

    std::cout << "Loaded " << faces.size() << " faces" << std::endl;

    Vector3f avg(0.f, 0.f, 0.f);
    for (int i=0; i<static_cast<int>(_lights.size()); i++) { //for each light

        Vector3f ctr(0.f, 0.f, 0.f);
        for (int tri=0; tri<_lights[i].n_triangles; tri++) { //for each face (triangle)

            for (int v=0; v<3; v++) {
                ctr += (_lights[i].faces[tri])[v]; //all vertices
            }
            ctr /= 3.0f; //centroid
            avg += ctr;

            //Heron's formula for area of arbitrary triangle
            float AB = (_lights[i].faces[tri][1] - _lights[i].faces[tri][0]).norm();
            float AC = (_lights[i].faces[tri][2] - _lights[i].faces[tri][0]).norm();
            float BC = (_lights[i].faces[tri][2] - _lights[i].faces[tri][1]).norm();

            float s = (AB + AC + BC)*0.5f;
            _lights[i].area += sqrt(s*(s-AB)*(s-AC)*(s-BC)); //accumulating area of entire light

//            _lights[i].area = (AB.cross(AC)).norm() / 2.f; //for now only since all are the same (area of 1 triangle)

        }
        _lights[i].avg_pos = avg / _lights[i].n_triangles;
    }

    Mesh *m = new Mesh;
    m->init(vertices,
            normals,
            uvs,
            colors,
            faces,
            materialIds,
            materials);
    m->setTransform(transform);

    return m;
}

const BVH &Scene::getBVH() const
{
    return *m_bvh;
}

const BasicCamera &Scene::getCamera() const
{
    return m_camera;
}

void Scene::setCamera(const BasicCamera &camera)
{
    m_camera = camera;
}

void Scene::setGlobalData(const CS123SceneGlobalData& data)
{
    m_globalData = data;
}

void Scene::addLight(const CS123SceneLightData &data)
{
    m_lights.push_back(data);
}

const std::vector<CS123SceneLightData> &Scene::getLights() const
{
    return m_lights;
}

void Scene::addPathLight(const PathLight &l) {
    _lights.push_back(l);
}

std::vector<PathLight> &Scene::getPathLights() {
    return _lights;
}

std::unordered_map<std::string, QImage> &Scene::getTextureMaps() {
    return _texmaps;
}

bool Scene::getIntersection(const Ray& ray, IntersectionInfo* I) const{
    return getBVH().getIntersection(ray, I, false);
}
