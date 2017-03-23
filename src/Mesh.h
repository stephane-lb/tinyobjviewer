//
// Created by stephan-lb on 23/03/2017.
//

#ifndef TINYOBJVIEWER_MESH_H
#define TINYOBJVIEWER_MESH_H

#include <string>
#include <Eigen/Sparse>
#include <vector>

#include "tiny_obj_loader.h"

typedef Eigen::Matrix<uint32_t, Eigen::Dynamic, Eigen::Dynamic> MatrixXu;

class Mesh {
  
  public:
  
  Mesh(const std::string &filename);
  
  ~Mesh();
  
  bool load_mesh(const std::string &filename);
  
  void export_mesh(const std::string &filename);
  
  unsigned int get_number_of_face();
  
  const Eigen::Vector3f get_mesh_center();
  
  const Eigen::MatrixXf *get_points();
  
  const MatrixXu *get_indices();
  
  const Eigen::MatrixXf *get_normals();
  
  float get_dist_max();
  
  private:
  
  std::vector<tinyobj::shape_t> m_shapes;
  std::vector<tinyobj::material_t> m_materials;
  
  size_t m_num_vertices;
  size_t m_num_faces;
  
  Eigen::Vector3f m_bmin;
  Eigen::Vector3f m_bmax;
  Eigen::Vector3f m_mesh_center;
  float m_dist_max = 0.0f;
  Eigen::MatrixXf m_points;
  MatrixXu m_indices;
  Eigen::MatrixXf m_normals;
};


#endif //TINYOBJVIEWER_MESH_H
