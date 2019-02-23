#ifndef SCENE_H
#define SCENE_H

#include <QString>

#include "BVH/BVH.h"

#include "basiccamera.h"

#include "util/CS123SceneData.h"

#include "shape/mesh.h"

#include <memory>


typedef struct  {
    int id;                                             //light ID
    int n_triangles = 0;                                //number of triangles/faces
    Eigen::Vector3f emission;                           //emitted energy (assuming uniform over all triangles)
    Eigen::Vector3f avg_pos;                            //average position of all triangles --- BAAAAAABE CHANGE THIS LATER TO BE MORE ACCURATE
    float area = 0;                                     //total area of light source
    float intensity;                                    //light intensity (magnitude of emission vector)
    std::vector<std::vector<Eigen::Vector3f>> faces;    //vector of all triangles of light (each containing 3 vertices)
} PathLight;


class Scene
{
public:

    Scene();
    virtual ~Scene();

    static bool load(QString filename, Scene **scenePointer);

    void setBVH(const BVH &bvh);
    const BVH& getBVH() const;

    const BasicCamera& getCamera() const;

    void setCamera(const BasicCamera& camera);
    void setGlobalData(const CS123SceneGlobalData& data);

    void addLight(const CS123SceneLightData& data);
    const std::vector<CS123SceneLightData>& getLights() const;

    void addPathLight(const PathLight &l);
    static std::vector<PathLight>& getPathLights();

    bool getIntersection(const Ray& ray, IntersectionInfo* I) const;


private:

    BVH *m_bvh;
    std::vector<Object *> *_objects;

    BasicCamera m_camera;

    CS123SceneGlobalData m_globalData;

    std::vector<CS123SceneLightData> m_lights;
    static bool parseTree(CS123SceneNode *root, Scene *scene, const std::string& baseDir);
    static void parseNode(CS123SceneNode *node, const Eigen::Affine3f &parentTransform, std::vector<Object *> *objects, const std::string& baseDir);
    static void addPrimitive(CS123ScenePrimitive *prim, const Eigen::Affine3f &transform, std::vector<Object *> *objects, const std::string& baseDir);
    static Mesh *loadMesh(std::string filePath, const Eigen::Affine3f &transform, const std::string& baseDir);
};

#endif // SCENE_H
