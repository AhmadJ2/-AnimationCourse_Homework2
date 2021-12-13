#include "tutorial/sandBox/sandBox.h"
#include "igl/edge_flaps.h"
#include "igl/collapse_edge.h"
#include "Eigen/dense"
#include <functional>
#include "igl/read_triangle_mesh.h"
#include "igl/parallel_for.h"
#include <vector>
#include <igl/vertex_triangle_adjacency.h>
#include "igl/infinite_cost_stopping_condition.h"
SandBox::SandBox()
{
	

}

void SandBox::Init(const std::string &config)
{
	using namespace std;
	using namespace Eigen;
	using namespace igl;
	cout << (config) << endl;;;
	std::string item_name;
	std::ifstream nameFileout;
	doubleVariable = 0;
	nameFileout.open(config);
	MatrixXd OV, VI;
	MatrixXi OF, FI;
	MatrixXi e(5, 5);
	e.row(1) = RowVectorXi(4);


	const auto& key_down =
		[&](igl::opengl::glfw::Viewer& viewer, unsigned char key, int mod)->bool
	{
		switch (key)
		{
		case ' ':
			exit(0);
			//viewer.core().is_animating ^= 1;
			break;
		case 'R':
		case 'r':
			//reset();
			break;
		default:
			return false;
		}
		return true;
	};

	int id = 0;
	std::vector<set<pair<double, int>>::iterator > Qit_;
	Matrix4d stam;
	stam = 0 * stam;
	for (int i = 0 ; i < 4 ; i++)
			stam(i, i) = 1;
	cout << stam.matrix() << endl;
	cout << stam.determinant() << endl;
	if (!nameFileout.is_open())
	{
		std::cout << "Can't open file "<<config << std::endl;
	}
	else
	{
		
		while (nameFileout >> item_name)
		{
			
			std::cout << "opening " << item_name << std::endl;
			load_mesh_from_file(item_name);
			read_triangle_mesh(item_name, OV, OF);
			parents.push_back(-1);
			data().add_points(Eigen::RowVector3d(0, 0, 0), Eigen::RowVector3d(0, 0, 1));
			data().show_overlay_depth = true;
			data().point_size = 10;
			data().line_width = 2;
			data().MyTranslate(Eigen::Vector3d(id ? 0.1 : -0.1, 0, 0), true);
			data().set_visible(false, 1);
			///////////////////////////////////////////////////////////////////////////////////
			init_objects_mesh_data_base(OV, OF);
			set<pair<double, int>> *Q_ = new set<pair<double, int>>();
			Q.push_back(Q_);
			C.push_back(new Eigen::MatrixXd(E[id].rows(), data().V.cols()));
			std::vector< PriorityQueue::iterator> QitTmp;
			QitTmp.resize(E[id].rows());
			for (int e = 0; e < E[id].rows(); e++)
			{
				double cost = 0;
				RowVectorXd p;									//faces->edges
																//EMAP
				lowest_error_cost(e, data().V, data().F, E[id], EMAP[id], EF[id], EI[id], cost, p,Qs[id]);
				C[id]->row(e) = p;
				QitTmp[e] = Q_->insert(std::pair<double, int>(cost, e)).first;
			}
			num_collapsed.push_back(0);
			Qit.push_back(QitTmp);
			++id;
		}
		nameFileout.close();
	}
	MyTranslate(Eigen::Vector3d(0, 0, -1), true);
	data().set_colors(Eigen::RowVector3d(0.9, 0.1, 0.1));

}
/*
	vector<VectorXi> EMAP;
	vector<MatrixXi> E, EF, EI;
	typedef set<pair<double, int> > PriorotyQueue;
	vector<PriorotyQueue*> Q;
	vector<vector<PriorotyQueue::iterator>> Qit;
	vector<MatrixXd*> C;
	vector<int> num_collapsed;
*/
void SandBox::init_objects_mesh_data_base(MatrixXd& V, MatrixXi& F)
{
	VectorXi EMAP_;
	MatrixXi E_, EF_, EI_;
	vector<Matrix4d> Qs_ (0);
	std::vector<std::vector<int> > VF_;
	std::vector<std::vector<int> > VFi_;
	igl::vertex_triangle_adjacency(V, F, VF_, VFi_);
	edge_flaps(F, E_, EMAP_, EF_, EI_);
	EMAP.push_back(EMAP_);
	E.push_back(E_);
	EF.push_back(EF_);
	EI.push_back(EI_);

	Qs_.resize(V.rows());
	for (int i = 0; i < Qs_.size(); i++) {
		Qs_[i] = Eigen::Matrix4d::Zero();
	}
	for (int v = 0; v < data().V.rows(); v++) {
		std::vector<int> v_faces = VF_[v];
		Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
		for (int f : v_faces) {
			auto n = data().F_normals.row(f).normalized();
			float d = 0;
			for (int j = 0; j < 3; j++) 
				d += (-n[j] * data().V.row(v)[j]);
			Eigen::Vector4d p(n[0], n[1], n[2], d);
			Q += p * p.transpose();
		}
		Qs_[v] = Q;
	}
	Qs.push_back(Qs_);
	/*
	cout << "V: " << V.size() << endl;
	cout << V << endl;
	cout << "F: " << F.size() << endl;
	cout << F << endl;
	cout << "E_: " << endl;
	cout << E_ << endl;
	cout << "EMAP_: " << EMAP_.size() << endl;
	int n = 0;
	for (int i = 0 ; i < EMAP_.size(); i++)
	{
		if (n % 3 == 0) cout << "<";
		cout << EMAP_[i] << ((n == 2) ? "" : ",");
		if (n % 3 == 2) cout << ">" << endl;
		n = (n + 1) % 3;
	}
	cout << EMAP_ << endl;
	cout << "EF_: " << endl;
	cout << EF_ << endl;
	cout << "EI_: " << endl;
	cout << EI_ << endl;
	*/
}

SandBox::~SandBox()
{

}

void SandBox::Animate()
{
	if (isActive)
	{
		
		
		
	}
}


