
#include "igl/opengl/glfw/renderer.h"
#include "tutorial/sandBox/inputManager.h"
#include "sandBox.h"
#include <igl/circulation.h>
#include <igl/collapse_edge.h>
#include <igl/edge_flaps.h>
#include <igl/decimate.h>
#include <igl/shortest_edge_and_midpoint.h>
#include <igl/parallel_for.h>
#include <igl/read_triangle_mesh.h>
#include <igl/opengl/glfw/Viewer.h>
#include <Eigen/Core>
#include <iostream>
#include <set>
int main(int argc, char* argv[])
{
  Display *disp = new Display(1200, 800, "Wellcome");
  Renderer renderer;

  SandBox viewer;
  
  igl::opengl::glfw::imgui::ImGuiMenu* menu = new igl::opengl::glfw::imgui::ImGuiMenu();
  viewer.Init("configuration.txt");
  for (int i = 0; i < 2; i++) {
	  viewer.selected_data_index = i;
	  renderer.core().toggle(viewer.data().show_lines);
	  viewer.data().set_colors(Eigen::RowVector3d(0.8, 0.8, 0));
	  viewer.data().build_alignment_box();
  }




  Init(*disp, menu);
  renderer.init(&viewer,2,menu);
  disp->SetRenderer(&renderer);
  disp->launch_rendering(true);
  delete menu;
  delete disp;
}
