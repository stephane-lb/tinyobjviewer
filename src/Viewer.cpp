//
// Created by stephan-lb on 22/03/2017.
//

#include "Viewer.h"
#include <algorithm>

Viewer::Viewer() : nanogui::Screen(Eigen::Vector2i(1024, 768), "TinyObjViewer") {
  
  m_window = new Window(this, "Controls");
  m_window->setPosition(Vector2i(15, 15));
  m_window->setLayout(new GroupLayout());
  
  Button *b = new Button(m_window, "Open mesh ...");
  b->setCallback([this]() {
    std::string filename = nanogui::file_dialog({{"obj", "Wavefront OBJ"}}, false);
    
    if (filename != "") {
      mProcessEvents = false;
      m_mesh = new Mesh(filename);
      this->refresh_mesh();
      this->refresh_trackball_center();
      mProcessEvents = true;
    }
    
  });
  
  performLayout();
  initShaders();

  m_mesh = new Mesh("/Users/Saorel/Downloads/teapot.obj");
  //m_mesh = new Mesh("/Users/Saorel/Downloads/metallic-lucy-statue-stanford-scan-obj-2/metallic-lucy-statue-stanford-scan.obj");
  this->refresh_mesh();
  this->refresh_trackball_center();
}

bool Viewer::keyboardEvent(int key, int scancode, int action, int modifiers) {
  if (Screen::keyboardEvent(key, scancode, action, modifiers)) {
    return true;
  }
  if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
    setVisible(false);
    return true;
  }
  return false;
}

void Viewer::draw(NVGcontext *ctx) {
  /* Draw the user interface */
  Screen::draw(ctx);
}

Vector2f Viewer::getScreenCoord() {
  Vector2i pos = mousePos();
  return Vector2f(2.0f * (float)pos.x() / width() - 1.0f,
                  1.0f - 2.0f * (float)pos.y() / height());
}

void Viewer::drawContents() {
  using namespace nanogui;
  
  if(m_mesh == nullptr)
    return;
  
  /* Draw the window contents using OpenGL */
  m_phong_shader.bind();
  
  Eigen::Matrix4f model, view, proj;
  computeCameraMatrices(model, view, proj);
  
  Matrix4f mv = view*model;
  Matrix4f p = proj;
  
  /* MVP uniforms */
  m_phong_shader.setUniform("MV", mv);
  m_phong_shader.setUniform("P", p);
  
  // Setup OpenGL (making sure the GUI doesn't disable these
  glEnable(GL_DEPTH_TEST);
  glDisable(GL_CULL_FACE);
  
  m_phong_shader.drawIndexed(GL_TRIANGLES, 0, m_mesh->get_number_of_face());
}

bool Viewer::scrollEvent(const Vector2i &p, const Vector2f &rel) {
  if (!Screen::scrollEvent(p, rel)) {
    m_camera.zoom = std::max(0.1, m_camera.zoom * (rel.y() > 0 ? 1.1 : 0.9));
  }
  return true;
}

bool Viewer::mouseMotionEvent(const Vector2i &p, const Vector2i &rel,
                              int button, int modifiers) {
  if (!Screen::mouseMotionEvent(p, rel, button, modifiers)) {
    if (m_camera.arcball.motion(p)) {
      //
    } else if (m_translate) {
      Eigen::Matrix4f model, view, proj;
      computeCameraMatrices(model, view, proj);
      Eigen::Vector3f mesh_center = m_mesh->get_mesh_center();
      float zval = nanogui::project(Vector3f(mesh_center.x(),
                                             mesh_center.y(),
                                             mesh_center.z()),
                                    view * model, proj, mSize).z();
      Eigen::Vector3f pos1 = nanogui::unproject(
          Eigen::Vector3f(p.x(), mSize.y() - p.y(), zval),
          view * model, proj, mSize);
      Eigen::Vector3f pos0 = nanogui::unproject(
          Eigen::Vector3f(m_translateStart.x(), mSize.y() -
                                               m_translateStart.y(), zval), view * model, proj, mSize);
      m_camera.modelTranslation = m_camera.modelTranslation_start + (pos1-pos0);
    }
  }
  return true;
}

bool Viewer::mouseButtonEvent(const Vector2i &p, int button, bool down, int modifiers) {
  if (!Screen::mouseButtonEvent(p, button, down, modifiers)) {
    if (button == GLFW_MOUSE_BUTTON_1 && modifiers == 0) {
      m_camera.arcball.button(p, down);
    } else if (button == GLFW_MOUSE_BUTTON_2 ||
               (button == GLFW_MOUSE_BUTTON_1 && modifiers == GLFW_MOD_SHIFT)) {
      m_camera.modelTranslation_start = m_camera.modelTranslation;
      m_translate = true;
      m_translateStart = p;
    }
  }
  if (button == GLFW_MOUSE_BUTTON_1 && !down) {
    m_camera.arcball.button(p, false);
  }
  if (!down) {
    m_translate = false;
  }
  return true;
}

void Viewer::initShaders() {
  // Shaders
  m_phong_shader.init(
      "a_simple_shader",
      
      /* Vertex shader */
      "#version 330\n"
          "uniform mat4 MV;\n"
          "uniform mat4 P;\n"
          
          "in vec3 position;\n"
          "in vec3 normal;\n"
          
          "out vec3 fcolor;\n"
          "out vec3 fnormal;\n"
          "out vec3 view_dir;\n"
          "out vec3 light_dir;\n"
          
          "void main() {\n"
          "    vec4 vpoint_mv = MV * vec4(position, 1.0);\n"
          "    gl_Position = P * vpoint_mv;\n"
          "    fcolor = vec3(0.7);\n"
          "    fnormal = mat3(transpose(inverse(MV))) * normal;\n"
          "    light_dir = vec3(0.0, 3.0, 3.0) - vpoint_mv.xyz;\n"
          "    view_dir = -vpoint_mv.xyz;\n"
          "}",
      
      /* Fragment shader */
      "#version 330\n"
          
          "in vec3 fcolor;\n"
          "in vec3 fnormal;\n"
          "in vec3 view_dir;\n"
          "in vec3 light_dir;\n"
          
          "out vec4 color;\n"
          
          "void main() {\n"
          "    vec3 c = vec3(0.0);\n"
          "    c += vec3(1.0)*vec3(0.18, 0.1, 0.1);\n"
          "    vec3 n = normalize(fnormal);\n"
          "    vec3 v = normalize(view_dir);\n"
          "    vec3 l = normalize(light_dir);\n"
          "    float lambert = dot(n,l);\n"
          "    if(lambert > 0.0) {\n"
          "        c += vec3(lambert);\n"
          "        vec3 v = normalize(view_dir);\n"
          "        vec3 r = reflect(-l,n);\n"
          "        c += vec3(pow(max(dot(r,v), 0.0), 90.0));\n"
          "    }\n"
          "    c *= fcolor;\n"
          "    color = vec4(c, 1.0);\n"
          "}"
  );
}

void Viewer::refresh_trackball_center() {
  // Re-center the mesh
  Eigen::Vector3f mesh_center = m_mesh->get_mesh_center();
  m_camera.arcball = Arcball();
  m_camera.arcball.setSize(mSize);
  m_camera.modelZoom = 2/m_mesh->get_dist_max();
  m_camera.modelTranslation = -Vector3f(mesh_center.x(), mesh_center.y(), mesh_center.z());
}

void Viewer::refresh_mesh() {
  m_phong_shader.bind();
  m_phong_shader.uploadIndices(*(m_mesh->get_indices()));
  m_phong_shader.uploadAttrib("position", *(m_mesh->get_points()));
  m_phong_shader.uploadAttrib("normal", *(m_mesh->get_normals()));
}

void Viewer::computeCameraMatrices(Eigen::Matrix4f &model,
                                   Eigen::Matrix4f &view,
                                   Eigen::Matrix4f &proj) {
  
  view = nanogui::lookAt(m_camera.eye, m_camera.center, m_camera.up);
  
  float fH = std::tan(m_camera.viewAngle / 360.0f * M_PI) * m_camera.dnear;
  float fW = fH * (float) mSize.x() / (float) mSize.y();
  
  proj = nanogui::frustum(-fW, fW, -fH, fH, m_camera.dnear, m_camera.dfar);
  model = m_camera.arcball.matrix();

  // model = nanogui::scale(model, CEigen::Vector3f::Constant(m_camera.zoom * m_camera.modelZoom));
  // model = nanogui::translate(model, m_camera.modelTranslation);
  
  model = model * nanogui::scale( Eigen::Vector3f::Constant(m_camera.zoom * m_camera.modelZoom));
  model = model * nanogui::translate(m_camera.modelTranslation);
}

Viewer::~Viewer() {
  m_phong_shader.free();
  delete m_mesh;
}