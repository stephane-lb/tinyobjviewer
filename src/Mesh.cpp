//
// Created by stephan-lb on 23/03/2017.
//

#include "Mesh.h"

#include <iostream>
#include <algorithm>
#include <Eigen/Dense>

Mesh::Mesh(const std::string &filename): m_num_vertices(0), m_num_faces(0)
{
  m_bmin =  Eigen::Vector3f(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
  m_bmax =  Eigen::Vector3f(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());
  m_mesh_center = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
  m_dist_max = 0.0f;
  
  load_mesh(filename);
}

Mesh::~Mesh() {
}

bool Mesh::load_mesh(const std::string &filename) {
  
  std::string err ;
  tinyobj::attrib_t attrib;
  bool load  = tinyobj::LoadObj(&attrib, &m_shapes, &m_materials, &err, filename.c_str());
  
  if (!load) {
    std::cerr << err << std::endl;
    return false;
  }
  
  if(!m_materials.empty())
  {
    std::cerr << "We are not supporting materials for the moments" << std::endl;
  }
  
  std::cout << "[LoadOBJ] # of shapes in .obj : " << m_shapes.size() << std::endl;
  std::cout << "[LoadOBJ] # of materials in .obj : " << m_materials.size() << std::endl;
  
  m_num_vertices = (int)(attrib.vertices.size()) / 3 ;
  
  for (size_t i = 0; i < m_shapes.size(); i++) {
    m_num_faces += m_shapes[i].mesh.indices.size() / 3;
  }
  
  std::cout << "[LoadOBJ] # of faces: " << m_num_faces << std::endl;
  std::cout << "[LoadOBJ] # of vertices: " << m_num_vertices << std::endl;
  
  m_points = Eigen::MatrixXf(3, m_num_vertices);
  m_indices = MatrixXu(3, m_num_faces);
  m_normals = Eigen::MatrixXf(3, m_num_vertices);
  
  for (size_t s = 0; s < m_shapes.size(); s++) {
    for (size_t f = 0; f < m_shapes[s].mesh.indices.size() / 3; f++) {
      tinyobj::index_t idx0 = m_shapes[s].mesh.indices[3 * f + 0];
      tinyobj::index_t idx1 = m_shapes[s].mesh.indices[3 * f + 1];
      tinyobj::index_t idx2 = m_shapes[s].mesh.indices[3 * f + 2];
  
      float v[3][3];
      for (int k = 0; k < 3; k++) {
        int f0 = idx0.vertex_index;
        int f1 = idx1.vertex_index;
        int f2 = idx2.vertex_index;
        assert(f0 >= 0);
        assert(f1 >= 0);
        assert(f2 >= 0);
        
        m_indices.col(f) << f0, f1, f2;

        v[0][k] = attrib.vertices[3 * f0 + k];
        v[1][k] = attrib.vertices[3 * f1 + k];
        v[2][k] = attrib.vertices[3 * f2 + k];
        
        m_bmin[k] = std::min(v[0][k], m_bmin[k]);
        m_bmin[k] = std::min(v[1][k], m_bmin[k]);
        m_bmin[k] = std::min(v[2][k], m_bmin[k]);
 
        m_bmax[k] = std::max(v[0][k], m_bmax[k]);
        m_bmax[k] = std::max(v[1][k], m_bmax[k]);
        m_bmax[k] = std::max(v[2][k], m_bmax[k]);
      }
      
      m_mesh_center = (m_bmax + m_bmin) / 2;
      m_dist_max = std::max( (m_bmax - m_mesh_center).norm(), (m_bmax - m_mesh_center).norm() );
  
      float n[3][3];
      if (attrib.normals.size() > 0) {
        int f0 = idx0.normal_index;
        int f1 = idx1.normal_index;
        int f2 = idx2.normal_index;
        assert(f0 >= 0);
        assert(f1 >= 0);
        assert(f2 >= 0);
        for (int k = 0; k < 3; k++) {
          n[0][k] = attrib.normals[3 * f0 + k];
          n[1][k] = attrib.normals[3 * f1 + k];
          n[2][k] = attrib.normals[3 * f2 + k];
        }
      } else {
        // compute geometric normal
        
        Eigen::Vector3f v0, v1, v2;
        v0 << v[0][0], v[0][1], v[0][2];
        v1 << v[1][0], v[1][1], v[1][2];
        v2 << v[2][0], v[2][1], v[2][2];
  
        Eigen::Vector3f v10 = v1 - v0;
        Eigen::Vector3f v20 = v2 - v0;
        
        Eigen::Vector3f N_face = v10.cross(v20);
        N_face.normalize();
  
        n[0][0] = N_face[0];
        n[0][1] = N_face[1];
        n[0][2] = N_face[2];
        n[1][0] = N_face[0];
        n[1][1] = N_face[1];
        n[1][2] = N_face[2];
        n[2][0] = N_face[0];
        n[2][1] = N_face[1];
        n[2][2] = N_face[2];
      }
  
      int f0 = idx0.vertex_index;
      int f1 = idx1.vertex_index;
      int f2 = idx2.vertex_index;
  
      m_points.col(f0) << v[0][0], v[0][1], v[0][2];
      m_points.col(f1) << v[1][0], v[1][1], v[1][2];
      m_points.col(f2) << v[2][0], v[2][1], v[2][2];
      
      m_normals.col(f0) << n[0][0], n[0][1], n[0][2];
      m_normals.col(f1) << n[1][0], n[1][1], n[1][2];
      m_normals.col(f2) << n[2][0], n[2][1], n[2][2];
    }
  }
  
  return true;
}

unsigned int Mesh::get_number_of_face() {
  return m_num_faces;
}

const Eigen::Vector3f Mesh::get_mesh_center() {
  return Eigen::Vector3f();
}

const Eigen::MatrixXf* Mesh::get_points() {
  return &m_points;
}

const MatrixXu *Mesh::get_indices() {
  return &m_indices;
}

const Eigen::MatrixXf *Mesh::get_normals() {
  return &m_normals;
}

float Mesh::get_dist_max()
{
  return m_dist_max;
}