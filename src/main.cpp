#include <iostream>
#include "Viewer.h"

/* Force usage of discrete GPU on laptops */
NANOGUI_FORCE_DISCRETE_GPU();

int main(int argc, char* argv[]) {
  
//  if (argc < 2) {
//    std::cout << "Needs input.obj\n" << std::endl;
//    return 0;
//  }
  
  try {
    nanogui::init();
    {
      nanogui::ref<Viewer> viewer = new Viewer();
      viewer->setVisible(true);
    
      nanogui::mainloop();
    }
    
    nanogui::shutdown();
  } catch (const std::runtime_error &e) {
    std::string error_msg = std::string("Caught a fatal error: ") + std::string(e.what());
    std::cerr << error_msg << std::endl;
    return -1;
  }
  
  
  return 0;
}
