//
// Created by stephan-lb on 22/03/2017.
//

#ifndef TINYOBJVIEWER_VIEWER_H
#define TINYOBJVIEWER_VIEWER_H

#include <nanogui/opengl.h>
#include <nanogui/glutil.h>
#include <nanogui/screen.h>
#include <nanogui/window.h>
#include <nanogui/layout.h>
#include <nanogui/popupbutton.h>
#include <nanogui/label.h>
#include <nanogui/button.h>
#include <nanogui/checkbox.h>
#include <nanogui/textbox.h>
#include <nanogui/tabwidget.h>
#include <nanogui/combobox.h>

#include "Mesh.h"

using namespace nanogui;

class Viewer : public nanogui::Screen {
  public:
  
  Viewer();
  ~Viewer();
  
  virtual void draw(NVGcontext *ctx);
  virtual void drawContents();
  
  void refresh_mesh();
  void refresh_trackball_center();
  
  virtual bool keyboardEvent(int key, int scancode, int action, int modifiers);
  
  Vector2f getScreenCoord();
  bool scrollEvent(const Vector2i &p, const Vector2f &rel);
  bool mouseMotionEvent(const Vector2i &p, const Vector2i &rel, int button, int modifiers);
  bool mouseButtonEvent(const Vector2i &p, int button, bool down, int modifiers);
  
  private:
  void initShaders();
  void computeCameraMatrices(Eigen::Matrix4f &model,
                             Eigen::Matrix4f &view,
                             Eigen::Matrix4f &proj);
  
  struct CameraParameters {
    nanogui::Arcball arcball;
    float zoom = 1.0f, viewAngle = 45.0f;
    float dnear = 0.05f, dfar = 100.0f;
    Eigen::Vector3f eye = Eigen::Vector3f(0.0f, 0.0f, 5.0f);
    Eigen::Vector3f center = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    Eigen::Vector3f up = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
    Eigen::Vector3f modelTranslation = Eigen::Vector3f::Zero();
    Eigen::Vector3f modelTranslation_start = Eigen::Vector3f::Zero();
    float modelZoom = 1.0f;
  };
  
  CameraParameters m_camera;
  bool m_translate = false;
  Vector2i m_translateStart = Vector2i(0, 0);
  
  // Variables for the viewer
  nanogui::GLShader m_phong_shader;
  nanogui::Window *m_window;
  
  Mesh* m_mesh;
};


#endif //TINYOBJVIEWER_VIEWER_H
