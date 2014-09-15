#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include "data_str/vector.h"
#include "data_str/pair.h"
#include "data_str/circular_buffer.h"
#include "data_str/hash_map_managed.h"
#include "data_str/hash_funcs.h"
#include "renderer/geometry/geometry_manager.h"
#include "renderer/geometry/geometry.h"
#include "renderer/geometry/geometry_colored_mesh.h"
#include "renderer/geometry/geometry_colored_boned_mesh.h"
#include "renderer/geometry/geometry_textured_mesh.h"
#include "renderer/geometry/geometry_textured_boned_mesh.h"
#include "renderer/colors.h"
#include "renderer/texture/texture.h"
#include "renderer/open_gl_common.h"
#include "assimp/Importer.hpp"      // C++ importer interface
#include "assimp/scene.h"           // Output data structure
#include "assimp/postprocess.h"     // Post processing flags
#include "math/math_types.h"
#include "fastlz/fastlz.h"

using jtil::math::Float3;
using jtil::math::Float4x4;
using jtil::math::FloatQuat;
using std::cout;
using std::endl;
using std::string;
using std::runtime_error;
using jtil::data_str::Vector;
using jtil::data_str::Pair;
using jtil::data_str::HashMapManaged;

#define SAFE_DELETE(target) \
  if (target != NULL) { \
    delete target; \
    target = NULL; \
  }

namespace renderer {

  GeometryManager* GeometryManager::g_geom_manager_;

  void GeometryManager::initGeometryManager() {
    if (g_geom_manager_ != NULL) {
      throw runtime_error(string("initGeometryManager() - ERROR: ") +
        string("GeometryManager is a singleton class.  init called twice!"));
    }
    g_geom_manager_ = new GeometryManager();
  }

  void GeometryManager::destroyGeometryManager() {
    if (g_geom_manager_ == NULL) {
      throw runtime_error(string("destroyGeometryManager() - ERROR: ") +
        string("GeometryManager was not initialized!"));
    }
    delete g_geom_manager_;
    g_geom_manager_ = NULL;
  }

  GeometryManager::GeometryManager() {
    tex_ = new HashMapManaged<std::string, Texture*>(GM_START_HM_SIZE, 
      jtil::data_str::HashString);
    scene_graph_root_ = new Geometry;
  }

  GeometryManager::~GeometryManager() {
    // Note HashMapManaged will clear all heap memory for us.
    delete tex_;

    // recursively delete the scene graph:
    SAFE_DELETE(scene_graph_root_);
  }

  Geometry* GeometryManager::loadFromFile(const string& path, 
    const string& filename, bool smooth_normals) {
    string full_path;
    if (path.at(path.length()-1) != '/' && path.at(path.length()-1) != '\\') {
      full_path = path + string("/");
    } else {
      full_path = path;
    }
    cout << "Loading scene from " << full_path << filename << "..." << endl;
    
     // Create an instance of the Importer class
    Assimp::Importer* importer = new Assimp::Importer();

    importer->SetPropertyInteger(AI_CONFIG_PP_LBW_MAX_WEIGHTS, MAX_VERTEX_BONE_COUNT);
    if (importer->GetPropertyInteger(AI_CONFIG_PP_LBW_MAX_WEIGHTS) != MAX_VERTEX_BONE_COUNT) {
      throw runtime_error(string("GeometryManager::loadFromFile() - ERROR: ") +
        string("Couldn't set the max bone weight property!"));
    }

    // And have it read the given file with some example postprocessing
    // List of post-processing options:
    // http://assimp.sourceforge.net/lib_html/postprocess_8h.html
    uint32_t flags = 0;
    flags |= aiProcess_Triangulate;  // In case faces have > 3 vertices
    flags |= aiProcess_ImproveCacheLocality;  // O(n)
    flags |= aiProcess_GenUVCoords;  // Convert sph or cyl to uv mappings
    flags |= aiProcess_OptimizeMeshes;
    flags |= aiProcess_OptimizeGraph;
    flags |= aiProcess_LimitBoneWeights;  // Small weights are ignored and then
                                          // weights are renormalized!
    if (smooth_normals) {
      flags |= aiProcess_JoinIdenticalVertices;  // Remove redundant data
      flags |= aiProcess_GenSmoothNormals;  // ignored if normals exist
    }

    if (!importer->ValidateFlags(flags)) {
      throw runtime_error(string("GeometryManager::loadFromFile() - ERROR: ") +
        string("One or more post-processing flag is not supported!"));
    }

    const aiScene* scene = importer->ReadFile(full_path + filename, flags);

    // If the import failed, report it
    if (!scene) {
      throw std::runtime_error(importer->GetErrorString());
    }

    // Now go through the assimp scene heirachy and copy to our scene format
    Geometry* geom = convertAssimpScene(full_path, filename, scene);
    if (geom == NULL) {
      cout << "GeometryManager::loadFromFile() - WARNING: Empty scene loaded from";
      cout << " file " << full_path << filename;
      geom = new Geometry();  // We might want to load in an empty geometry!
    }

    // Now, that all the geometry cells are loaded in, we need to find the 
    // named geometry elements corresponding to mesh bones (if they exist)
    associateBoneTransforms(geom, filename);

    // Clean up the assimp structures.
    delete importer;

    cout << "Finished loading " << full_path << filename << endl;

    return geom;
  }

  Geometry* GeometryManager::convertAssimpScene(const string& path, 
      const string& filename, const aiScene* scene) {
    Geometry* root = NULL;

    // Make sure the scene has meshes and material, and if so, recurse down
    // the heirachy
    if (scene->HasMaterials() && scene->HasMeshes() && scene->mRootNode) {
      root = convertAssimpNode(path, filename, scene, scene->mRootNode, root);
    }

    return root;
  }

  Geometry* GeometryManager::convertAssimpNode(const string& path, 
      const string& filename, const aiScene* scene, 
    const aiNode* node, Geometry* parent) {
    Geometry* geom;

    // From: http://assimp.sourceforge.net/lib_html/data.html
    // 1.  Meshes don't live in the aiNode structure, but in the aiScene
    // structure.  The nodes just store indices into the aiScene's mesh array.
    // 2. A single mesh uses a single material type.
    if (node->mNumMeshes == 1) {
      // A Single mesh --> Just create a single container to simplify heir.
      geom = convertAssimpMesh(path, filename, scene, 
        scene->mMeshes[node->mMeshes[0]]);
    } else if (node->mNumMeshes > 1) {
      // Create an implicit root node and add the mulitple meshes as children
      geom = new Geometry();
      for (uint32_t i = 0; i < node->mNumMeshes; i++) {
        Geometry* child = convertAssimpMesh(path, filename, scene, 
          scene->mMeshes[node->mMeshes[i]]);
        if (child != NULL) {
          geom->addChild(child);
        }
      }
    } else {
      // Blank node --> These are often used for bone transforms.
      geom = new Geometry();
    }

    geom->name_ = std::string(node->mName.data);

    // assimp uses row major layout so we may need to transpose
    const float* mat_assimp = node->mTransformation[0];
    Float4x4* mat = geom->mat();
    memcpy(mat->m, mat_assimp, 16 * sizeof(mat->m[0]));
#ifdef COLUMN_MAJOR
    mat->transpose();
#endif

    // Recurse down the structure
    for (uint32_t i = 0; i < node->mNumChildren; i++) {
      Geometry* child = convertAssimpNode(path, filename, scene, 
        node->mChildren[i], geom);
      if (child != NULL) {
        geom->addChild(child);
      }
    }

    return geom;
  }

  Geometry* GeometryManager::convertAssimpMesh(const string& path, 
      const string& filename, const aiScene* scene, 
    const aiMesh* mesh) {
    GeometryType type = extractGeometryType(mesh);

    Geometry* geom;
    switch (type) {
    case GeometryType::GEOMETRY_COLORED_MESH:
      geom = GeometryColoredMesh::convertAssimpMesh(path, filename, scene, 
        mesh);
      break;
    case GeometryType::GEOMETRY_COLORED_BONED_MESH:
      geom = GeometryColoredBonedMesh::convertAssimpMesh(path, filename, scene, 
        mesh);
      break;
    case GeometryType::GEOMETRY_TEXTURED_MESH:
      geom = GeometryTexturedMesh::convertAssimpMesh(path, filename, scene, 
        mesh);
      break;
    case GeometryType::GEOMETRY_TEXTURED_BONED_MESH:
      geom = GeometryTexturedBonedMesh::convertAssimpMesh(path, filename, scene, 
        mesh);
      break;
    default:
      throw std::runtime_error("convertAssimpMesh() - ERROR: unrecognized type");
    }

    return geom;
  }

  void GeometryManager::saveToJFile(Geometry* root, const string& path, 
    const string& filename) {
    uint32_t n_nodes = root->numNodes();

    string full_path;
    if (path.at(path.length()-1) != '/' && path.at(path.length()-1) != '\\') {
      full_path = path + string("/") + filename;
    } else {
      full_path = path + filename;
    }

    std::ofstream file(full_path.c_str(), std::ios::out | std::ios::binary);
    if (!file.is_open()) {
      throw std::runtime_error(string("error opening file:") + full_path);
    }

    // Write the total number of nodes in the file first
    file.write((const char*)(&n_nodes), sizeof(n_nodes));
    file.flush();

    // Now save the whole heirachy to file BFS and compress each node as we go
    jtil::data_str::CircularBuffer<Geometry*> queue(n_nodes+1);
    queue.write(root);  // Push this node to the back of the empty queue
    while(!queue.empty()) {
      Geometry* cur_node;
      queue.read(cur_node);
      
      // Add the node's children to the queue for processing
      for (uint32_t i = 0; i < cur_node->numChildren(); i++) {
        queue.write(cur_node->getChild(i));
      }

      // Convert the node to a data array:
      // The pair is 1. the data and 2. the length of the data
      Pair<uint8_t*,uint32_t> cur_node_data = cur_node->saveToArray();
      
      // Compress the current node:
      // (Malloc 5% extra space in case the node isn't compressable!)
      void* compressed_data = malloc((cur_node_data.second * 21) / 20);
      static const int compression_level = 1;  // 1 fast, 2 better compression
      int compressed_length = fastlz_compress_level(compression_level, 
        reinterpret_cast<void*>(cur_node_data.first), 
        cur_node_data.second * sizeof(cur_node_data.first[0]),
        reinterpret_cast<void*>(compressed_data));

      // Now save the node to disk along with the data we need to recover the
      // heirachy later:
      // 1. Number of children
      uint32_t num_children = cur_node->numChildren();
      file.write((const char*)(&num_children), sizeof(num_children));
      file.flush();
      // 2. The compressed data length
      file.write((const char*)(&compressed_length), sizeof(compressed_length));
      file.flush();
      // 3. The uncompressed data length
      file.write((const char*)(&cur_node_data.second), sizeof(cur_node_data.second));
      file.flush();
      // 4. The node data itself
      file.write(reinterpret_cast<const char*>(compressed_data), compressed_length);
      file.flush();
      
      // We're done with all the temporary data so delete it to save room
      free(compressed_data);
      free(cur_node_data.first);
    }

    // Collect all the bones to save to disk:

    Vector<BoneFileInfo*> bones;
    std::map<std::string, uint32_t> bones_map;
    render_stack_.resize(0);
    render_stack_.pushBack(root);
    while(!renderStackEmpty()) {
      Geometry* cur_geom = renderStackPop();
      BoneFileInfo* bi = NULL;
      if (cur_geom->type() == GeometryType::GEOMETRY_COLORED_BONED_MESH) {
        GeometryColoredBonedMesh* g = reinterpret_cast<GeometryColoredBonedMesh*>(cur_geom);
        bi = g->bones_;
      } else if (cur_geom->type() == GeometryType::GEOMETRY_TEXTURED_BONED_MESH) {
        GeometryTexturedBonedMesh* g = reinterpret_cast<GeometryTexturedBonedMesh*>(cur_geom);
        bi = g->bones_;
      }
      if (bi != NULL) {
        if (bones_map.find(bi->model_filename) == bones_map.end()) {
          // First time we've come accross this bone file info, insert it
          uint32_t cur_bi_index = bones.size();
          bones_map[bi->model_filename] = cur_bi_index;
          bones.pushBack(bi);
        }
      }
    }

    uint32_t num_bone_file_info = bones.size();  // Usually going to be one
    file.write((const char*)(&num_bone_file_info), sizeof(num_bone_file_info));
    file.flush();

    // Now save each bone data:
    for (uint32_t i = 0; i < bones.size(); i++) {
      Pair<uint8_t*,uint32_t> cur_node_data = bones[i]->saveToArray();

      // Compress the current bone:
      // (Malloc 5% extra space in case the node isn't compressable!)
      void* compressed_data = malloc((cur_node_data.second * 21) / 20);
      static const int compression_level = 1;  // 1 fast, 2 better compression
      int compressed_length = fastlz_compress_level(compression_level, 
        reinterpret_cast<void*>(cur_node_data.first), 
        cur_node_data.second * sizeof(cur_node_data.first[0]),
        reinterpret_cast<void*>(compressed_data));

      // Now save the node to disk along with the data we need to recover the
      // heirachy later:
      // 1. The compressed data length
      file.write((const char*)(&compressed_length), sizeof(compressed_length));
      file.flush();
      // 2. The uncompressed data length
      file.write((const char*)(&cur_node_data.second), sizeof(cur_node_data.second));
      file.flush();
      // 3. The node data itself
      file.write(reinterpret_cast<const char*>(compressed_data), compressed_length);
      file.flush();
      
      // We're done with all the temporary data so delete it to save room
      free(compressed_data);
      free(cur_node_data.first);
    }

    file.close();
  }

  Geometry* GeometryManager::readNodeData(const string& path, 
      const string& filename, std::ifstream& in_file) {
    uint32_t num_children;
    in_file.read(reinterpret_cast<char*>(&num_children), sizeof(num_children));
    int compressed_length;
    in_file.read(reinterpret_cast<char*>(&compressed_length), 
      sizeof(compressed_length));
    Pair<uint8_t*,uint32_t> cur_node_data;
    in_file.read(reinterpret_cast<char*>(&cur_node_data.second), 
      sizeof(cur_node_data.second));

    uint8_t* compressed_data = (uint8_t*)malloc(compressed_length);
    uint8_t* uncompressed_data = (uint8_t*)malloc(cur_node_data.second);

    in_file.read(reinterpret_cast<char*>(compressed_data), compressed_length);

    int size_decompress = fastlz_decompress(
      reinterpret_cast<void*>(compressed_data), compressed_length,
      reinterpret_cast<void*>(uncompressed_data), cur_node_data.second);
    if (size_decompress != cur_node_data.second) {
      throw runtime_error(string("readNodeData() - ERROR: ") +
        string("uncompressed data size is not what we expected!"));
    }

    // The beginning of the uncompressed data contains an enum indicating what
    // type of geometry block this is.
    Geometry* ret_val;
    uint32_t enum_val = *((uint32_t*)uncompressed_data);
    switch(enum_val) {
    case GeometryType::GEOMETRY_BASE:
      ret_val = new Geometry();
      ret_val->loadFromArray(path, filename, uncompressed_data);
      break;
    case GeometryType::GEOMETRY_COLORED_MESH:
      ret_val = new GeometryColoredMesh();
      ret_val->loadFromArray(path, filename, uncompressed_data);
      break;
    case GeometryType::GEOMETRY_TEXTURED_MESH:
      ret_val = new GeometryTexturedMesh();
      ret_val->loadFromArray(path, filename, uncompressed_data);
      break;
    case GeometryType::GEOMETRY_TEXTURED_BONED_MESH:
      ret_val = new GeometryTexturedBonedMesh();
      ret_val->loadFromArray(path, filename, uncompressed_data);
      break;
    case GeometryType::GEOMETRY_COLORED_BONED_MESH:
      ret_val = new GeometryColoredBonedMesh();
      ret_val->loadFromArray(path, filename, uncompressed_data);
      break;
    default:
      throw runtime_error(string("loadFromJFile() - ERROR:") +
        string(" file enum value is not recognized"));
    }

    ret_val->children_.capacity(num_children);

    free(compressed_data);
    free(uncompressed_data);

    return ret_val;
  }

  BoneFileInfo* GeometryManager::readBoneData(const string& path, 
      const string& filename, std::ifstream& in_file) {
    int compressed_length;
    in_file.read(reinterpret_cast<char*>(&compressed_length), 
      sizeof(compressed_length));
    Pair<uint8_t*,uint32_t> cur_node_data;
    in_file.read(reinterpret_cast<char*>(&cur_node_data.second), 
      sizeof(cur_node_data.second));

    uint8_t* compressed_data = (uint8_t*)malloc(compressed_length);
    uint8_t* uncompressed_data = (uint8_t*)malloc(cur_node_data.second);

    in_file.read(reinterpret_cast<char*>(compressed_data), compressed_length);

    int size_decompress = fastlz_decompress(
      reinterpret_cast<void*>(compressed_data), compressed_length,
      reinterpret_cast<void*>(uncompressed_data), cur_node_data.second);
    if (size_decompress != cur_node_data.second) {
      throw runtime_error(string("readNodeData() - ERROR: ") +
        string("uncompressed data size is not what we expected!"));
    }

    // The beginning of the uncompressed data contains an enum indicating what
    // type of geometry block this is.
    BoneFileInfo* ret_val = new BoneFileInfo();
    ret_val->loadFromArray(path, filename, uncompressed_data);

    free(compressed_data);
    free(uncompressed_data);

    return ret_val;
  }

  Geometry* GeometryManager::loadFromJFile(const string& path, 
      const string& filename) {
    string full_path;
    if (path.at(path.length()-1) != '/' && path.at(path.length()-1) != '\\') {
      full_path = path + string("/");
    } else {
      full_path = path;
    }
    cout << "Loading object from " << full_path << filename << "..." << endl;
    string path_filename = full_path + filename;
    std::ifstream in_file(path_filename.c_str(), 
      std::ios::in | std::ios::binary | std::ios::ate);
    if (!in_file.is_open()) {
      throw std::runtime_error(string("loadFromJFile()") + 
        string(": error opening file") + path_filename);
    }
    in_file.seekg(0, std::ios::beg);

    // First comes the number of nodes
    uint32_t n_nodes;
    in_file.read(reinterpret_cast<char*>(&n_nodes), sizeof(n_nodes));

    // Now read in the nodes one, by one.  The nodes are stored BFS.
    Geometry* root = readNodeData(path, filename, in_file);
    jtil::data_str::CircularBuffer<Geometry*> queue(n_nodes+1);
    queue.write(root);  // Push this node to the back of the empty queue

    while(!queue.empty()) {
      Geometry* cur_node;
      queue.read(cur_node);

      uint32_t num_children = cur_node->children_.capacity();
      for (uint32_t i = 0; i < num_children; i++) {
        Geometry* child = readNodeData(path, filename, in_file);
        cur_node->addChild(child);
        queue.write(child);
      }
    }

    // Now load in the bone data:
    uint32_t n_bones;
    in_file.read(reinterpret_cast<char*>(&n_bones), sizeof(n_bones));
    Vector<BoneFileInfo*> new_bones;
    for (uint32_t i = 0; i < n_bones; i++) {
      BoneFileInfo* bi = readBoneData(path, filename, in_file);
      bi->model_root_node = root;
      uint32_t bone_filename_index = bones_.size();
      bones_.pushBack(bi);
      bones_filename_index_map_[bi->model_filename] = bone_filename_index;
      new_bones.pushBack(bi);
    }

    // Now, that all the geometry cells are loaded in, we need to find the 
    // named geometry elements corresponding to mesh bones (if they exist)
    for (uint32_t i = 0; i < new_bones.size(); i++) {
      associateBoneTransforms(root, new_bones[i]->model_filename);
    }

    cout << "Finished loading from " << path_filename << endl;

    return root;
  }

  void GeometryManager::renderStackReset() {
    render_stack_.resize(0);  // empty the stack (without deallocating)
    // Seed the render stack with the root node
    render_stack_.pushBack(scene_graph_root_);
  }

  Geometry* GeometryManager::renderStackPop() {
    Geometry* ret = NULL;
    if (render_stack_.size() > 0) {
      render_stack_.popBackUnsafe(ret);  // Remove the last element

      // Now add the children to the geometry stack
      for (uint32_t i = 0; i < ret->numChildren(); i ++) {
        render_stack_.pushBack(ret->getChild(i));
      }
    }
    return ret;
  }

  bool GeometryManager::renderStackEmpty() {
    return render_stack_.size() == 0;
  }

  Texture* GeometryManager::loadTexture(const std::string& path_filename) {
    // First see if the texture has already been loaded in.
    Texture* ret_tex;
    if (!tex_->lookup(path_filename.c_str(), ret_tex)) {
      ret_tex = new Texture(path_filename, TEXTURE_WRAP_MODE::TEXTURE_CLAMP);
      tex_->insert(path_filename.c_str(), ret_tex);
    }

    return ret_tex;
  }
  
  // associateBoneTransforms - Potentially O(n^2) --> But this will be VERY 
  // rare and is done once at startup.
  void GeometryManager::associateBoneTransforms(Geometry* root, 
    const std::string& filename) {
    // See if any bone data exists for this file (it may not)
    BoneFileInfo* bones_in_file = findBoneFileInfo(filename);
    
    if (bones_in_file == NULL) {
      return;
    }

    bones_in_file->model_root_node = root;

    // Iterate through the bone list and find all the nodes it attaches to
    for (uint32_t i = 0; i < bones_in_file->bones.size(); i++) {
      Bone* cur_bone = bones_in_file->bones[i];
      Geometry* node = findGeometryByName(root, cur_bone->name);  // O(n)
      if (node == NULL) {
        throw runtime_error(string("GeometryManager::associateBone") +
          string("Transforms() - ERROR: Couldn't find bone geometry!"));
      }
      cur_bone->node = node;
    }

    // Now fix the pointers from the geometry elements that point to BoneFileInfo
    render_stack_.resize(0);
    render_stack_.pushBack(root);
    while(!renderStackEmpty()) {
      Geometry* geom = renderStackPop();
      BoneFileInfo** bi = NULL;
      if (geom->type() == GeometryType::GEOMETRY_COLORED_BONED_MESH) {
        bi = &((GeometryColoredBonedMesh*)geom)->bones_;
      }
      if (geom->type() == GeometryType::GEOMETRY_TEXTURED_BONED_MESH) {
        bi = &((GeometryTexturedBonedMesh*)geom)->bones_;
      }
      if (bi != NULL) {
        *bi = bones_in_file;
      }
    }
  }

  Geometry* GeometryManager::findGeometryByName(Geometry* node, string& name) {
    if (node->name_ == name) {
      return node;
    }
    // Otherwise check our children:
    for (uint32_t i = 0; i < node->numChildren(); i++) {
      Geometry* return_node = findGeometryByName(node->getChild(i), name);
      if (return_node != NULL) {
        return return_node;
      }
    }
    // Otherwise no subtree has the node, so return NULL
    return NULL;
  }

  Geometry* GeometryManager::findGeometryByType(Geometry* node,
    GeometryType type) {
    if ((int)node->type() == (int)type) {
      return node;
    }
    // Otherwise check our children:
    for (uint32_t i = 0; i < node->numChildren(); i++) {
      Geometry* return_node = findGeometryByType(node->getChild(i), type);
      if (return_node != NULL) {
        return return_node;
      }
    }
    // Otherwise no subtree has the node, so return NULL
    return NULL;
  }

  GeometryType GeometryManager::extractGeometryType(const aiMesh* mesh) {
    // Figure out if the mesh is textured
    aiString Path;
    bool textured = mesh->HasTextureCoords(0);

    // TO DO: Include textured mesh and bones
    if (mesh->HasNormals() && mesh->HasFaces() && !mesh->HasBones() &&
      !textured) {
      return GeometryType::GEOMETRY_COLORED_MESH;
    } else if (mesh->HasNormals() && mesh->HasFaces() && mesh->HasBones() &&
      !textured) {
      return GeometryType::GEOMETRY_COLORED_BONED_MESH;
    } else if (mesh->HasNormals() && mesh->HasFaces() && !mesh->HasBones() &&
      textured) {
      return GeometryType::GEOMETRY_TEXTURED_MESH;
    } else if (mesh->HasNormals() && mesh->HasFaces() && mesh->HasBones() &&
      textured) {
      return GeometryType::GEOMETRY_TEXTURED_BONED_MESH;
    } else {
      std::stringstream ss;
      ss << "convertAssimpMesh() - WARNING: Mesh type not supported by this";
      ss << " rendering engine:" << endl;
      ss << "  --> HasNormals() = " << mesh->HasNormals() << endl;
      ss << "  --> HasFaces() = " << mesh->HasFaces() << endl;
      ss << "  --> HasBones() = " << mesh->HasBones() << endl;
      ss << "  --> textured = " << textured << endl;
      throw std::runtime_error(ss.str());
    }
  }

  Bone* GeometryManager::findBone(const std::string& file, 
    const std::string& bone_name) {
    BoneFileInfo* bones_in_file = findBoneFileInfo(file);

    if (bones_in_file != NULL) {
      return bones_in_file->findBone(bone_name);
    }
    // If we got to here then no bone exists
    return NULL;
  }

  BoneFileInfo* GeometryManager::findBoneFileInfo(const std::string& file) {
    if (bones_filename_index_map_.find(file) != 
      bones_filename_index_map_.end()) {
      BoneFileInfo* bones_in_file = bones_[bones_filename_index_map_[file]];
      return bones_in_file;
    } else {
      return NULL;
    }
  }

  void GeometryManager::insertBone(const std::string& file, 
    const std::string& bone_name, Bone* bone) {
    BoneFileInfo* bones_in_file = findBoneFileInfo(file);

    if (bones_in_file == NULL) {
      bones_in_file = new BoneFileInfo();
      bones_in_file->model_filename = file;
      bones_in_file->model_root_node = NULL;
      uint32_t bone_filename_index = bones_.size();
      bones_.pushBack(bones_in_file);
      bones_filename_index_map_[file] = bone_filename_index;
    }

    Bone* b = bones_in_file->findBone(bone_name);
    if (b != NULL) {
      throw runtime_error(string("GeometryManager::insertBone() - ERROR") + 
        string(": bone already exists!"));
    }

    // create a new index for this bone.
    bone->index = bones_in_file->bones.size();
    bones_in_file->bone_mapping[bone_name] = bone->index;
    bones_in_file->bones.pushBack(bone);
  }
  
  void GeometryManager::updateBoneMatrices() {
    // Assumption: The entire heirachy of matricies has already been updated 
    // (ie, the per-node matricies have been multiplied by their parent's down
    // the tree. 
    // Therefore, we only need to add the local bone transform to each node.
    for (uint32_t i = 0; i < bones_.size(); i++) {
      BoneFileInfo* cur_file = bones_[i];
      updateBoneMatrices(cur_file);
    }
  }

  void GeometryManager::updateBoneMatrices(BoneFileInfo* bones) {
    Float4x4 bone_offset;
    Float4x4 global_inverse_transform;
    Float4x4 tmp;
    FloatQuat rot;
    Float3 scale, trans;
    // Update the global transform inverse:
    Float4x4::inverse(global_inverse_transform, 
      *bones->model_root_node->mat_hierarchy());

    // Now update all the bones at once
    for (uint32_t j = 0; j < bones->bones.size(); j++) {
      Bone* cur_bone = bones->bones[j];
      Float4x4* cur_bone_final_trans = &cur_bone->final_trans;
      // Annoying O(16) copy, but making bone_offset float[16] is much easier
      // for saving it to disk (windows vs mac os x template size is different)
      bone_offset.set(cur_bone->bone_offset);  
      Float4x4::multSIMD(tmp, *cur_bone->getNode()->mat_hierarchy(), bone_offset);
      Float4x4::multSIMD(*cur_bone_final_trans, global_inverse_transform, tmp);

#ifndef LINEAR_BLEND_SKINNING
      // FloatQuat::decompose(&bones_[i].final_trans, &trans, &rot, &scale);

      // Decomposition for just translation scale matricies --> FAST, but
      // scale must be <1, 1, 1>:
      FloatQuat::decompose(cur_bone_final_trans, &trans, &rot);
      FloatQuat::quatTrans2UnitDualQuat(&rot, &trans, 
        cur_bone->uniform_dual_quaternion);
#endif
    }
  }

}  // namespace renderer

