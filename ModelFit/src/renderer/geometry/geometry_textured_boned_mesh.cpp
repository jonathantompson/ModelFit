#include <iostream>
#include <map>
#include "renderer/geometry/geometry_textured_boned_mesh.h"
#include "renderer/geometry/geometry_manager.h"
#include "renderer/objects/aabbox.h"
#include "renderer/texture/texture.h"
#include "math/math_types.h"
#include "renderer/colors.h"
#include "assimp/Importer.hpp"      // C++ importer interface
#include "assimp/scene.h"           // Output data structure
#include "assimp/postprocess.h"     // Post processing flags
#include "fastlz/fastlz.h"
#include "string_util/string_util.h"
#include "data_str/pair.h"
#include "renderer/gl_state.h"

#ifdef max
  #undef max
#endif
#define max std::max

using jtil::math::Float3;
using jtil::math::Float2;
using jtil::math::FloatQuat;
using jtil::math::Float4x4;
using renderer::objects::AABBox;
using jtil::data_str::Pair;
using std::string;
using std::runtime_error;
using std::string;
using std::runtime_error;
using std::endl;
using std::cout;
using std::map;

namespace renderer {
  const void* TexturedBonedMeshVertex::pos_offset = 
    reinterpret_cast<const void*>(offsetof(struct TexturedBonedMeshVertex, pos));
  const void* TexturedBonedMeshVertex::norm_offset = 
    reinterpret_cast<const void*>(offsetof(struct TexturedBonedMeshVertex, norm));
  const void* TexturedBonedMeshVertex::tex_offset = 
    reinterpret_cast<const void*>(offsetof(struct TexturedBonedMeshVertex, tex));
  const void* TexturedBonedMeshVertex::bone_ids_03_offset = 
    reinterpret_cast<const void*>(offsetof(struct TexturedBonedMeshVertex, bone_data.ids_03));
  const void* TexturedBonedMeshVertex::bone_weights_03_offset = 
    reinterpret_cast<const void*>(offsetof(struct TexturedBonedMeshVertex, bone_data.weights_03));
  //const void* TexturedBonedMeshVertex::bone_ids_47_offset = 
  //  reinterpret_cast<const void*>(offsetof(struct TexturedBonedMeshVertex, bone_data.ids_47));
  //const void* TexturedBonedMeshVertex::bone_weights_47_offset = 
  //  reinterpret_cast<const void*>(offsetof(struct TexturedBonedMeshVertex, bone_data.weights_47));

  GeometryTexturedBonedMesh::GeometryTexturedBonedMesh() :
  Geometry() {
    // vertices_ will be zero size by default
    // colors_ will be zero size by default
    synced_ = false;
    tex_ = NULL;  // NOT OWNED HERE
    aabbox_ = new AABBox();
  }

  GeometryTexturedBonedMesh::~GeometryTexturedBonedMesh() {
    if (synced_) {
      GLState::glsDeleteBuffers(1, &vbo_);
      GLState::glsDeleteVertexArrays(1, &vao_);
    }
    if (aabbox_) {
      delete aabbox_;
      aabbox_ = NULL;
    }

    children_.clear();  // Explicitly delete all the children (calling destrs)
  }

  void GeometryTexturedBonedMesh::unsyncVAO() {
    if (synced_) {
      GLState::glsDeleteBuffers(1, &vbo_);
      GLState::glsDeleteVertexArrays(1, &vao_);
      synced_ = false;
    } else {
      throw runtime_error("unsyncVAO() - Geometry not yet synced!");
    }
  }

  void GeometryTexturedBonedMesh::syncVAO() {
    if (synced_) {
      string err("syncVBO() - ERROR: dynamic VBOs not yet supported");
      throw std::runtime_error(err);
    }

    if (texture_coords_.size() != vertices_.size() || 
        texture_coords_.size() != normals_.size() ||
        texture_coords_.size() != vertex_bone_data_.size()) {
      string err = string("syncVBO() - ERROR: vertex, texture bones and "
        "normal vectors must be the same size!");
      throw std::runtime_error(err);
    }

    if (indices_.size() == 0) {
      if ((vertices_.size() % 3) != 0) {
        string err = string("syncVBO() - ERROR: num vertices must be a "
          "multiple of 3 when not using the index array");
        throw std::runtime_error(err);
      }
    }

    synced_num_vertices_ = vertices_.size();

    // Allocate a VAO
    GLState::glsGenVertexArrays(1, &vao_);
    // Bind our Vertex Array Object as the current used object
    GLState::glsBindVertexArray(vao_);
    // Allocate and assign two Vertex Buffer Objects to our handle
    GLState::glsGenBuffers(1, &vbo_);
    // Bind our first VBO as being the active buffer and storing vertex 
    // attributes (coordinates)
    GLState::glsBindBuffer(GL_ARRAY_BUFFER, vbo_);

    // Allocate a vertex buffer
    TexturedBonedMeshVertex dummy;  // just for sizeof
    static_cast<void>(dummy);  // Get rid of unreference local variable warning
    GLState::glsBufferData(GL_ARRAY_BUFFER,  // Target
                 vertices_.size() * sizeof(dummy),  // size
                 NULL,  // data --> Data is not initially copied
                 GL_STATIC_DRAW);  // usage

    // Copy the data into the vertex buffer
    TexturedBonedMeshVertex* ptr = reinterpret_cast<TexturedBonedMeshVertex*>(
      GLState::glsMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY));
    for (uint32_t i = 0; i < vertices_.size(); i++) {
      ptr[i].pos[0] = static_cast<GLfloat>(vertices_[i][0]);
      ptr[i].pos[1] = static_cast<GLfloat>(vertices_[i][1]);
      ptr[i].pos[2] = static_cast<GLfloat>(vertices_[i][2]);
      ptr[i].norm[0] = static_cast<GLfloat>(normals_[i][0]);
      ptr[i].norm[1] = static_cast<GLfloat>(normals_[i][1]);
      ptr[i].norm[2] = static_cast<GLfloat>(normals_[i][2]);
      ptr[i].tex[0] = static_cast<GLfloat>(texture_coords_[i][0]);
      ptr[i].tex[1] = static_cast<GLfloat>(texture_coords_[i][1]);
      for (uint32_t j = 0; j < 4; j++) {
        ptr[i].bone_data.ids_03[j] = vertex_bone_data_[i].ids_03[j];
        ptr[i].bone_data.weights_03[j] = vertex_bone_data_[i].weights_03[j];
        //ptr[i].bone_data.ids_47[j] = vertex_bone_data_[i].ids_47[j];
        //ptr[i].bone_data.weights_47[j] = vertex_bone_data_[i].weights_47[j];
      }
    }
    GLState::glsUnmapBuffer(GL_ARRAY_BUFFER);

    setVertexAttribPointer(VERTEX_POS_LOC, 3, GL_FLOAT, false, 
      sizeof(struct TexturedBonedMeshVertex), TexturedBonedMeshVertex::pos_offset);
    setVertexAttribPointer(VERTEX_NOR_LOC, 3, GL_FLOAT, false,
      sizeof(struct TexturedBonedMeshVertex), TexturedBonedMeshVertex::norm_offset);
    setVertexAttribPointer(VERTEX_TEX_LOC, 2, GL_FLOAT, false,
      sizeof(struct TexturedBonedMeshVertex), TexturedBonedMeshVertex::tex_offset);
    setVertexAttribIPointer(VERTEX_BONE_IDS_03_LOC, 4, GL_INT,
      sizeof(struct TexturedBonedMeshVertex), TexturedBonedMeshVertex::bone_ids_03_offset);
    setVertexAttribPointer(VERTEX_BONE_WEIGHTS_03_LOC, 4, GL_FLOAT,
      false, sizeof(struct TexturedBonedMeshVertex), 
      TexturedBonedMeshVertex::bone_weights_03_offset);
    //setVertexAttribIPointer(VERTEX_BONE_IDS_47_LOC, 4, GL_INT,
    //  sizeof(struct TexturedBonedMeshVertex), TexturedBonedMeshVertex::bone_ids_47_offset);
    //setVertexAttribPointer(VERTEX_BONE_WEIGHTS_47_LOC, 4, GL_FLOAT,
    //  false, sizeof(struct TexturedBonedMeshVertex), 
    //  TexturedBonedMeshVertex::bone_weights_47_offset);

    // Allocate an index buffer if we're using it
    if (indices_.size() != 0) {
      GLState::glsGenBuffers(1, &ibo_);
      GLState::glsBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo_);

      // Allocate the space for the index buffer
      GLState::glsBufferData(GL_ELEMENT_ARRAY_BUFFER, 
        indices_.size()*sizeof(indices_[0]), indices_.at(0), GL_STATIC_DRAW);

      synced_num_indices_ = indices_.size();
    } else {
      ibo_ = 0;
      synced_num_indices_ = 0;
    }

    aabbox_->init(&vertices_);

    synced_ = true;
  }

  void GeometryTexturedBonedMesh::bindVAO() {
    GLState::glsBindVertexArray(vao_);  // encapsulates all the vbo bindings
    ERROR_CHECK;
  }

  void GeometryTexturedBonedMesh::unbindVAO() {
    // Nothing to do
  }

  void GeometryTexturedBonedMesh::draw() {
    if (!synced_) {
      throw std::runtime_error(string("GeometryTexturedBonedMesh::draw() - ") + 
        string("ERROR: trying to draw an unsynced vbo object!"));
    }

    bindVAO();  // Bind all required OpenGL resources
    if (synced_num_indices_ == 0) {  // We're drawing without an index buffer
      GLState::glsDrawArrays(GL_TRIANGLES, 0, synced_num_vertices_);
      ERROR_CHECK;
    } else {
      GLState::glsDrawElements(GL_TRIANGLES, synced_num_indices_, GL_UNSIGNED_INT, 
        static_cast<GLvoid*>(NULL));
    }
    unbindVAO();  // Unbind all required OpenGL resources
  }

  GeometryTexturedBonedMesh* GeometryTexturedBonedMesh::convertAssimpMesh(
    const std::string& path, const std::string& filename,
    const aiScene* scene, const aiMesh* mesh) {
    GeometryTexturedBonedMesh* geom = new GeometryTexturedBonedMesh();

    if (!mesh->HasTextureCoords(0)) {
      throw runtime_error(string("convertAssimpMesh() - ERROR: aiMesh does") +
        string(" not contain any texture coordinates!"));
    }

    aiMaterial* assimp_mtrl = scene->mMaterials[mesh->mMaterialIndex];

    /*
    float spec_pow;
    if (AI_SUCCESS == assimp_mtrl->Get(AI_MATKEY_SHININESS, spec_pow)) {
      mtrl->specular_power = spec_pow;
    }

    float spec_intens;
    if (AI_SUCCESS == 
      assimp_mtrl->Get(AI_MATKEY_SHININESS_STRENGTH, spec_intens)) {
      mtrl->specular_intensity = spec_intens;
    }
    */

    if (!mesh->HasNormals() || !mesh->HasFaces()) {
      throw runtime_error(string("Geometry::convertAssimpMesh() - ") + 
        string("ERROR: mesh does not have normals or faces!"));
    }

    if (!mesh->HasBones()) {
      throw runtime_error(string("convertAssimpMesh() - ERROR: aiMesh does") +
        string(" not contain any bones!"));
    }

    geom->vertices_.capacity(mesh->mNumVertices);
    geom->vertices_.resize(mesh->mNumVertices);
    geom->normals_.capacity(mesh->mNumVertices);
    geom->normals_.resize(mesh->mNumVertices);
    geom->texture_coords_.capacity(mesh->mNumVertices);
    geom->texture_coords_.resize(mesh->mNumVertices);
    for (uint32_t i = 0; i < mesh->mNumVertices; i++) {
      geom->vertices_[i][0] = mesh->mVertices[i].x;
      geom->vertices_[i][1] = mesh->mVertices[i].y;
      geom->vertices_[i][2] = mesh->mVertices[i].z;

      geom->normals_[i][0] = mesh->mNormals[i].x;
      geom->normals_[i][1] = mesh->mNormals[i].y;
      geom->normals_[i][2] = mesh->mNormals[i].z;

      // ASSUME FOR NOW THAT THE CORRECT TEXTURE IS IN INDEX 0
      // THERE IS A BETTER WAY TO DO THIS!
      geom->texture_coords_[i][0] = mesh->mTextureCoords[0][i].x;
      geom->texture_coords_[i][1] = mesh->mTextureCoords[0][i].y;
    }

    geom->indices_.capacity(mesh->mNumFaces * 3);
    geom->indices_.resize(mesh->mNumFaces * 3);
    for (uint32_t i = 0; i < mesh->mNumFaces; i++) {
      aiFace face = mesh->mFaces[i];
      if (face.mNumIndices != 3) {
        throw runtime_error(string("Geometry::convertAssimpMesh() - ") + 
          string("ERROR: Found a face that isn't 3 verticies!"));
      }
      // Assimp indicies are wound the wrong way..., so reverse the order of
      // last two.
      geom->indices_[i*3] = face.mIndices[0];
      geom->indices_[i*3+1] = face.mIndices[2];
      geom->indices_[i*3+2] = face.mIndices[1];
    }

    // Load in the bones (the following tutorial was helpful!
    // http://ogldev.atspace.co.uk/www/tutorial38/tutorial38.html)

    // Zero all the vertex weights:
    geom->vertex_bone_data_.capacity(mesh->mNumVertices);
    geom->vertex_bone_data_.resize(mesh->mNumVertices);
    for (uint32_t i = 0; i < mesh->mNumVertices; i++) {
      for (uint32_t j = 0; j < 4; j++) {
        geom->vertex_bone_data_[i].ids_03[j] = 0;
        geom->vertex_bone_data_[i].weights_03[j] = 0.0f;
        //geom->vertex_bone_data_[i].ids_47[j] = 0;
        //geom->vertex_bone_data_[i].weights_47[j] = 0.0f;
      }
    }

    Float4x4 mat;
    uint32_t num_bones = 0;
    GeometryManager* geom_man = GeometryManager::g_geom_manager();
    for (uint32_t i = 0; i < mesh->mNumBones; i++) {
      string bone_name = string(mesh->mBones[i]->mName.data);

      // See if the bone exists in the global bone database:
      Bone* bone = geom_man->findBone(filename, bone_name);
      if (bone == NULL) {
        // Bone doesn't yet exist, so we need to create one and insert it
        bone = new Bone(bone_name, mesh->mBones[i]->mOffsetMatrix[0]);
        // Ownership is transfered to the GeometryManager!
        geom_man->insertBone(filename, bone_name, bone);
      }

      num_bones++;
      uint32_t bone_index = bone->index;  // Index in the shader bone arr

      for (uint32_t j = 0; j < mesh->mBones[i]->mNumWeights ; j++) {
        uint32_t vertex = mesh->mBones[i]->mWeights[j].mVertexId;
        float weight = mesh->mBones[i]->mWeights[j].mWeight;
        // Use the next avaliable bone attachement (that is zero weight)
        geom->vertex_bone_data_[vertex].attachBone(bone_index, weight);
      }
    }
    geom->bones_ = geom_man->findBoneFileInfo(filename);
    geom->bones_filename_ = filename;

    // Load in the texture
    geom->tex_ = NULL;
    if (assimp_mtrl->GetTextureCount(aiTextureType_DIFFUSE) > 0) {
      aiString str;
      if (assimp_mtrl->GetTexture(aiTextureType_DIFFUSE, 0, &str, NULL, NULL, 
        NULL, NULL, NULL) == AI_SUCCESS) {
        std::string FullPath = path + string(str.data);
        geom->tex_ = GeometryManager::g_geom_manager()->loadTexture(FullPath);
        geom->texture_filename_ = string(str.data);
      }
    }
    if (geom->tex_ == NULL) {
      // Texture doesn't exist.  Load in a blank white texture.
      geom->tex_ = GeometryManager::g_geom_manager()->loadTexture("./textures/white.tga");
      geom->texture_filename_ = string("./textures/white.tga");
    }

    geom->syncVAO();

    cout << "    - loaded GeometryTexturedBonedMesh with ";
    cout << geom->synced_num_vertices_ << " vertices and ";
    cout << geom->synced_num_indices_ << " faces" << endl;
    cout << "      texture file:";
    cout << geom->tex()->filename() << endl;
    cout << "      number of bones:" << num_bones << endl;

    return geom;
  }

  Pair<uint8_t*,uint32_t> GeometryTexturedBonedMesh::saveToArray() {
    Pair<uint8_t*,uint32_t> data;
    data.first = NULL;
    data.second = 0;

    char char_dummy;
    float float_dummy;
    static_cast<void>(char_dummy);
    static_cast<void>(float_dummy);
    if (sizeof(char_dummy) != 1 || sizeof(float_dummy) != 4) {
      throw std::runtime_error("saveToArray - basic types are the wrong size!");
    }

    uint32_t data_size = TEXTURED_BONED_MESH_FILE_DATA_SIZE +
      (uint32_t)(name_.size() + 1) * sizeof(char_dummy) +  // name
      (uint32_t)(texture_filename_.size() + 1) * sizeof(char_dummy) +  // Filename
      (uint32_t)(bones_filename_.size() + 1) * sizeof(char_dummy) +  // bone filename
      vertices_.size() * sizeof(float_dummy) * 3 +  // vertices
      normals_.size() * sizeof(float_dummy) * 3 +  // normals
      texture_coords_.size() * sizeof(float_dummy) * 2 +  // UV coords
      vertex_bone_data_.size() * BONE_VERTEX_DATA_FILE_DATA_SIZE +  // bone data
      indices_.size() * 4;  // indices - 32bit uint x 1
    data.second = data_size;

    data.first = (uint8_t*)malloc(data_size);

    // Get the data ready
    TexturedBonedMeshFileData* preamble =
      reinterpret_cast<TexturedBonedMeshFileData*>(data.first);
    preamble->type = GeometryType::GEOMETRY_TEXTURED_BONED_MESH;
    memcpy(preamble->mat_m, mat_.m, 16 * sizeof(preamble->mat_m[0]));
    preamble->mtrl_specular_intensity = mtrl_.specular_intensity;
    preamble->mtrl_specular_power = mtrl_.specular_power;
    preamble->num_ind = indices_.size();
    preamble->num_vert = vertices_.size();
    preamble->file_size = (uint32_t)texture_filename_.size();
    preamble->name_size = (uint32_t)name_.size();
    preamble->bone_filename_size = (uint32_t)bones_filename_.size();

    // Now copy the name string
    char* name_c_str = reinterpret_cast<char*>(&data.first[TEXTURED_BONED_MESH_FILE_DATA_SIZE]);
    strcpy(name_c_str, name_.c_str());

    char* filename_c_str = &name_c_str[name_.size()+1];
    strcpy(filename_c_str, texture_filename_.c_str());

    char* bone_filename_c_str = &filename_c_str[texture_filename_.size()+1];
    strcpy(bone_filename_c_str, bones_filename_.c_str());

    float* vert = reinterpret_cast<float*>(&bone_filename_c_str[bones_filename_.size()+1]);
    for (uint32_t i = 0; i < vertices_.size(); i++) {
      vert[i * 3] = vertices_[i][0];
      vert[i * 3 + 1] = vertices_[i][1];
      vert[i * 3 + 2] = vertices_[i][2];
    }
    
    float* norm = &vert[vertices_.size() * 3];
    for (uint32_t i = 0; i < normals_.size(); i++) {
      norm[i * 3] = normals_[i][0];
      norm[i * 3 + 1] = normals_[i][1];
      norm[i * 3 + 2] = normals_[i][2];
    }

    float* tex = &norm[normals_.size() * 3];
    for (uint32_t i = 0; i < texture_coords_.size(); i++) {
      tex[i * 2] = texture_coords_[i][0];
      tex[i * 2 + 1] = texture_coords_[i][1];
    }

    uint8_t* bone_data = reinterpret_cast<uint8_t*>(&tex[texture_coords_.size() *2]);
    for (uint32_t i = 0; i < vertex_bone_data_.size(); i++) {
      VertexBoneData* cur_bone_data = 
        (VertexBoneData*)(&bone_data[BONE_VERTEX_DATA_FILE_DATA_SIZE * i]);
      for (uint32_t j = 0; j < 4; j++) {
        cur_bone_data->ids_03[j] = vertex_bone_data_[i].ids_03[j];
        //cur_bone_data->ids_47[j] = vertex_bone_data_[i].ids_47[j];
        cur_bone_data->weights_03[j] = vertex_bone_data_[i].weights_03[j];
        //cur_bone_data->weights_47[j] = vertex_bone_data_[i].weights_47[j];
      }
    }

    uint32_t* ind = (uint32_t*)(&bone_data[BONE_VERTEX_DATA_FILE_DATA_SIZE * 
                                           vertex_bone_data_.size()]);
    for (uint32_t i = 0; i < indices_.size(); i++) {
      ind[i] = indices_[i];
    }

    return data;
  }

  void GeometryTexturedBonedMesh::loadFromArray(const std::string& path, 
      const std::string& filename, const uint8_t* arr) {
    const TexturedBonedMeshFileData* preamble = 
      reinterpret_cast<const TexturedBonedMeshFileData*>(arr);

    if (preamble->type != GeometryType::GEOMETRY_TEXTURED_BONED_MESH) {
      throw runtime_error(string("GeometryTexturedBonedMesh::loadFromArray() - ") +
        string("INTERNAL ERROR: Incorrect data type"));
    }

    mat_.set((float*)preamble->mat_m);
    mtrl_.specular_intensity = preamble->mtrl_specular_intensity;
    mtrl_.specular_power = preamble->mtrl_specular_power;

    // Extract the name
    const char* name_c_str = (const char*)(&arr[TEXTURED_BONED_MESH_FILE_DATA_SIZE]); 
    name_ = string(name_c_str);

    const char* filename_c_str = &name_c_str[preamble->name_size + 1];
    texture_filename_ = string(filename_c_str);

    const char* bones_filename_c_str = &filename_c_str[preamble->file_size + 1];
    bones_filename_ = string(bones_filename_c_str);

    const float* vert = reinterpret_cast<const float*>(&bones_filename_c_str[preamble->bone_filename_size + 1]);
    vertices_.capacity(preamble->num_vert);
    vertices_.resize(preamble->num_vert);
    for (uint32_t i = 0; i < preamble->num_vert; i++) {
      vertices_[i].set(vert[i * 3], vert[i * 3 + 1], vert[i * 3 + 2]);
    }

    const float* norm = &vert[preamble->num_vert *3 ];
    normals_.capacity(preamble->num_vert);
    normals_.resize(preamble->num_vert);
    for (uint32_t i = 0; i < preamble->num_vert; i++) {
      normals_[i].set(norm[i * 3], norm[i * 3 + 1], norm[i * 3 + 2]);
    }

    const float* tex = &norm[preamble->num_vert * 3];
    texture_coords_.capacity(preamble->num_vert);
    texture_coords_.resize(preamble->num_vert);
    for (uint32_t i = 0; i < preamble->num_vert; i++) {
      texture_coords_[i].set(tex[i * 2], tex[i * 2 + 1]);
    }

    const uint8_t* bone_data = (const uint8_t*)(&tex[preamble->num_vert *2]);
    vertex_bone_data_.capacity(preamble->num_vert);
    vertex_bone_data_.resize(preamble->num_vert);
    for (uint32_t i = 0; i < preamble->num_vert; i++) {
      VertexBoneData* cur_bone_data = 
        (VertexBoneData*)(&bone_data[BONE_VERTEX_DATA_FILE_DATA_SIZE * i]);
      for (uint32_t j = 0; j < 4; j++) {
        vertex_bone_data_[i].ids_03[j] = cur_bone_data->ids_03[j];
        //vertex_bone_data_[i].ids_47[j] = cur_bone_data->ids_47[j];
        vertex_bone_data_[i].weights_03[j] = cur_bone_data->weights_03[j];
        //vertex_bone_data_[i].weights_47[j] = cur_bone_data->weights_47[j];
      }
    }

    const uint32_t* ind = 
      (const uint32_t*)(&bone_data[preamble->num_vert * BONE_VERTEX_DATA_FILE_DATA_SIZE]);
    indices_.capacity(preamble->num_ind);
    indices_.resize(preamble->num_ind);
    for (uint32_t i = 0; i < preamble->num_ind; i++) {
      indices_[i] = ind[i];
    }

    // Load in the texture
    std::string FullPath = path + texture_filename_;
    tex_ = GeometryManager::g_geom_manager()->loadTexture(FullPath);

    syncVAO();
    cout << "    - loaded GeometryTexturedBonedMesh with ";
    cout << synced_num_vertices_ << " vertices and ";
    cout << synced_num_indices_ << " faces" << endl;
    cout << "      texture file:";
    cout << tex_->filename() << endl;
  }

  Geometry* GeometryTexturedBonedMesh::copy() {
    GeometryTexturedBonedMesh* ret = new GeometryTexturedBonedMesh();
    ret->mat_ = mat_;
    ret->mtrl_ = mtrl_;
    ret->vertices_ = vertices_;
    ret->normals_ = normals_;
    ret->texture_coords_ = texture_coords_;
    ret->indices_ = indices_;
    ret->bones_ = bones_;
    ret->vertex_bone_data_ = vertex_bone_data_;
    ret->bones_ = bones_;

    for (uint32_t i = 0; i < children_.size(); i++) {
      Geometry* child = children_[i]->copy();
      ret->addChild(child);
    }
    ret->tex_ = tex_->copy();
    ret->syncVAO();
    return ret;
  }
}  // namespace renderer

