//
//  geometry_manager.h
//
//  Created by Jonathan Tompson on 11/2/12.
//
//  Singleton class.
//
//  Geometry IO and resource Managment. Uses ASSIMP to load in 3D models in 
//  many possible formats (includeing: 3ds, x, Collada, etc).  However, while
//  this library is flexible it is slow.
//
//  Instead, I have implemented a custom "jbin" format, which is a very simple 
//  but very fast to load compressed binary format.  The load store memory 
//  overhead is high and will need to be fixed later.
//
//  NOTE: WHEN GEOMETRY IS ADDED TO THE SCENE GRAPH ROOT, THE MEMORY OWNERSHIP
//        IS TRANSFERED TO THE GEOMETRYMANAGER CLASS (IT WILL HANDLE 
//        DESTRUCTION)

#pragma once

#include <map>
#include "math/math_types.h"
#include "data_str/vector.h"
#include "data_str/vector_managed.h"
#include "geometry.h"  // For GeometryType

struct aiScene;
struct aiNode;
struct aiMesh;

#define GM_START_HM_SIZE 211  // Starting hash-map size.  Best if it is prime.

namespace jtil {
namespace data_str { template <typename TFirst, typename TSecond> class Pair; }
namespace data_str { template <class TKey, class TValue> class HashMapManaged; }
}

namespace renderer {

  class Texture;
  struct Bone;
  struct BoneFileInfo;

  class GeometryManager {
  public:
    static void initGeometryManager();
    static void destroyGeometryManager();

    inline static GeometryManager* g_geom_manager() { return g_geom_manager_; }
    inline static Geometry* scene_graph_root() { return g_geom_manager_->scene_graph_root_; }

    // loadFromFile - Slow.  Uses ASSIMP library to parse many format types
    Geometry* loadFromFile(const std::string& path, 
      const std::string& filename, bool smooth_normals = true);

    // loadFromJFile - Fast and compact.  Use loadFromFile then saveToJFile to 
    //                 convert to this format.
    Geometry* loadFromJFile(const std::string& path, 
      const std::string& filename);

    // saveToFile - Some memory overhead
    //              Saves the entire geometry tree to disk!
    void saveToJFile(Geometry* root, const std::string& path, 
      const std::string& filename);

    // If it's already loaded then find it in the database, otherwise load
    // in a new texture.
    Texture* loadTexture(const std::string& path_filename);

    // Insertion and querying of bone data into the global database.
    Bone* findBone(const std::string& file, const std::string& bone_name);
    void insertBone(const std::string& file, const std::string& bone_name, 
      Bone* bone); 
    // findBoneFileInfo - Find all bones attributed to a datafile (may span
    // multiple geometry elements)
    BoneFileInfo* findBoneFileInfo(const std::string& file);

    // updateBoneMatrices -> Must be called after geometry heirachy has been
    // desended and the parent x child matrix multiply has been performed.
    void updateBoneMatrices();
    void updateBoneMatrices(BoneFileInfo* bones);  // Update a specific bone set only

    // Render Stack inteface methods
    void renderStackReset();
    Geometry* renderStackPop();
    bool renderStackEmpty();

    // findGeometryByName - O(N) DFS Recursive search for named geometry
    Geometry* findGeometryByName(Geometry* node, std::string& name);
    // findGeometryByType - O(N) DFS Recursive search, return first element of type
    Geometry* findGeometryByType(Geometry* node, GeometryType type); 

  private:
    static GeometryManager* g_geom_manager_;

    GeometryManager();
    ~GeometryManager();

    // Textures for all GeometryTexturedMesh and GeometryTexturedBonedMesh are
    // stored here (so we can avoid loading in the same texture twice).
    jtil::data_str::HashMapManaged<std::string, Texture*>* tex_;

    // Bones may be shared accross multiple geometry elements (different meshes
    // use the same bones for instance).  Bones are stored first by the 
    // filename to which they belong, then in a container of the bones for that
    // file.
    // We need O(1) string lookup but also we need to linearly iterate through 
    // the database, so we need BOTH a hash map and a vector.
    jtil::data_str::VectorManaged<BoneFileInfo*> bones_;
    std::map<std::string, uint32_t> bones_filename_index_map_;

    static uint32_t strHashFunc(uint32_t size, const char* key);

    // The global scene graph used by the renderer
    Geometry* scene_graph_root_;
    jtil::data_str::Vector<Geometry*> render_stack_;

    Geometry* convertAssimpScene(const std::string& path, 
      const std::string& filename, const aiScene* scene);
    Geometry* convertAssimpNode(const std::string& path, 
      const std::string& filename, const aiScene* scene, 
      const aiNode* node, Geometry* parent);
    Geometry* convertAssimpMesh(const std::string& path, 
      const std::string& filename, const aiScene* scene, 
      const aiMesh* mesh);
    static Geometry* readNodeData(const std::string& path, 
      const std::string& filename, std::ifstream& in_file);
    static BoneFileInfo* readBoneData(const std::string& path, 
      const std::string& filename, std::ifstream& in_file);

    // associateBoneTransforms - Finds the nodes that bones attach to by their
    // string name.
    void associateBoneTransforms(Geometry* root, const std::string& filename);

    GeometryType extractGeometryType(const aiMesh* mesh);

    // Non-copyable, non-assignable.
    GeometryManager(GeometryManager&);
    GeometryManager& operator=(const GeometryManager&);
  };
};  // renderer namespace
