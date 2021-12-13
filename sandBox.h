#pragma once
#include "igl/opengl/glfw/Viewer.h"
#include "igl/aabb.h"
#include <igl/shortest_edge_and_midpoint.h>

using namespace std;
using namespace Eigen;
using namespace igl;
class SandBox : public igl::opengl::glfw::Viewer
{
public:
	SandBox();
	~SandBox();
	void Init(const std::string& config);
	void init_objects_mesh_data_base(MatrixXd& V, MatrixXi& F);
	double doubleVariable;
	


	// Prepare array-based edge data structures and priority queue
	vector<vector<Matrix4d>> Qs;
	
	vector<VectorXi> EMAP;
	vector<MatrixXi> E, EF, EI;
	typedef  set<pair<double, int>> PriorityQueue;
	vector<set<pair<double, int> >*> Q;
	vector<vector<PriorityQueue::iterator>> Qit;
	vector<MatrixXd*> C;
	vector<int> num_collapsed;
	///////////////////////////////////////////////////////////////
	
	void Animate();
};

