
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimbodyEngine/PlanarJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/PinJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/WeldJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/Joint.h>
#include <OpenSim/Simulation/SimbodyEngine/SpatialTransform.h>
#include <OpenSim/Simulation/SimbodyEngine/CustomJoint.h>
#include <OpenSim/Common/LinearFunction.h>
#include <OpenSim/Common/Constant.h>
#include <OpenSim/Common/SimmSpline.h>
#include <OpenSim/Simulation/Model/ConditionalPathPoint.h>
#include <OpenSim/Simulation/Model/MovingPathPoint.h>
#include <OpenSim/Simulation/Model/HuntCrossleyForce_smooth.h>
#include <recorder.hpp>
#include <initializer_list>
#include <unordered_map>

#include <iostream>
#include <iterator>
#include <random>
#include <cassert>
#include <algorithm>
#include <vector>
#include <fstream>

using namespace SimTK;
using namespace OpenSim;

/// number of inputs/outputs of function F
constexpr int n_in = 3;
constexpr int n_out = 1;

constexpr int NDOF = 37; // nDOF
constexpr int NX = NDOF * 2; // states
constexpr int NU = NDOF; // controls (accelerations) 
constexpr int NP = 24; // contact parameters + vt, mu_s, mu_d, mu_v
constexpr int NR = NDOF; // generalized forces (torques)

						 /* Function F, using templated type T
						 F(x,u,p) -> (Tau)
						 */

std::unordered_map<std::string, int>
createSystemYIndexMap(const Model& model) {
	std::unordered_map<std::string, int> sysYIndices;
	auto s = model.getWorkingState();
	const auto svNames = model.getStateVariableNames();
	s.updY() = 0;
	for (int iy = 0; iy < s.getNY(); ++iy) {
		s.updY()[iy] = SimTK::NaN;
		const auto svValues = model.getStateVariableValues(s);
		for (int isv = 0; isv < svNames.size(); ++isv) {
			if (SimTK::isNaN(svValues[isv])) {
				sysYIndices[svNames[isv]] = iy;
				s.updY()[iy] = 0;
				break;
			}
		}
	}
	SimTK_ASSERT2_ALWAYS(svNames.size() == (int)sysYIndices.size(),
		"Expected to find %i state indices but found %i.", svNames.size(),
		sysYIndices.size());
	return sysYIndices;
}

Vec3 crossproduct(SimTK::Vec3 a, SimTK::Vec3 b)
{
	Vec3 c;
	c.setToZero();

	c[0] = a[1] * b[2] - a[2] * b[1];
	c[1] = a[2] * b[0] - a[0] * b[2];
	c[2] = a[0] * b[1] - a[1] * b[0];

	return c;
}

template<typename T>
T value(const Recorder& e) {
	return e;
}

template<>
double value(const Recorder& e) {
	return e.getValue();
}

template<typename T>
int F_generic(const T** arg, T** res) {

	// Read inputs
	std::vector<T> x(arg[0], arg[0] + NX);
	std::vector<T> u(arg[1], arg[1] + NU);
	std::vector<T> p(arg[2], arg[2] + NP);

	T contactPrms[NP];
	T ua[NDOF];
	//T ugrf[36];

	OpenSim::Model* model;
	OpenSim::Body* pelvis;
	OpenSim::Body* femur_r;
	OpenSim::Body* tibia_r;
	OpenSim::Body* talus_r;
	OpenSim::Body* calcn_r;
	OpenSim::Body* toes_r;
	OpenSim::Body* femur_l;
	OpenSim::Body* tibia_l;
	OpenSim::Body* talus_l;
	OpenSim::Body* calcn_l;
	OpenSim::Body* toes_l;
	OpenSim::Body* torso;
	OpenSim::Body* humerus_r;
	OpenSim::Body* ulna_r;
	OpenSim::Body* radius_r;
	OpenSim::Body* hand_r;
	OpenSim::Body* humerus_l;
	OpenSim::Body* ulna_l;
	OpenSim::Body* radius_l;
	OpenSim::Body* hand_l;

	OpenSim::CustomJoint* ground_pelvis;
	OpenSim::CustomJoint* hip_r;
	OpenSim::CustomJoint* knee_r;
	OpenSim::CustomJoint* ankle_r;
	OpenSim::CustomJoint* subtalar_r;
	OpenSim::CustomJoint* mtp_r;
	OpenSim::CustomJoint* hip_l;
	OpenSim::CustomJoint* knee_l;
	OpenSim::CustomJoint* ankle_l;
	OpenSim::CustomJoint* subtalar_l;
	OpenSim::CustomJoint* mtp_l;
	OpenSim::CustomJoint* back;
	OpenSim::CustomJoint* acromial_r;
	OpenSim::CustomJoint* elbow_r;
	OpenSim::CustomJoint* radioulnar_r;
	OpenSim::CustomJoint* radius_hand_r;
	OpenSim::CustomJoint* acromial_l;
	OpenSim::CustomJoint* elbow_l;
	OpenSim::CustomJoint* radioulnar_l;
	OpenSim::CustomJoint* radius_hand_l;

	model = new OpenSim::Model();
	pelvis = new OpenSim::Body("pelvis", 11.3078, Vec3(-0.062385, 0, 0), Inertia(0.075601, 0.075601, 0.043286, 0, 0, 0));
	model->addBody(pelvis);
	//
	femur_r = new OpenSim::Body("femur_r", 8.9308, Vec3(0, -0.18157, 0), Inertia(0.13984, 0.024801, 0.13984, 0, 0, 0));
	model->addBody(femur_r);
	//
	tibia_r = new OpenSim::Body("tibia_r", 3.5598, Vec3(0, -0.20889, 0), Inertia(0.059175, 0.0033172, 0.059175, 0, 0, 0));
	model->addBody(tibia_r);
	//
	talus_r = new OpenSim::Body("talus_r", 0.096016, Vec3(0, 0, 0), Inertia(0.0007392, 0.00070393, 0.00070393, 0, 0, 0));
	model->addBody(talus_r);
	//
	calcn_r = new OpenSim::Body("calcn_r", 1.2002, Vec3(0.083451, 0.026323, 0), Inertia(0.0010349, 0.0026571, 0.0026571, 0, 0, 0));
	model->addBody(calcn_r);
	//
	toes_r = new OpenSim::Body("toes_r", 0.20797, Vec3(0.028874, 0.0052645, -0.015355), Inertia(7.392e-05, 0.00013726, 0.00013726, 0, 0, 0));
	model->addBody(toes_r);
	//
	femur_l = new OpenSim::Body("femur_l", 8.9308, Vec3(0, -0.18157, 0), Inertia(0.13984, 0.024801, 0.13984, 0, 0, 0));
	model->addBody(femur_l);
	//
	tibia_l = new OpenSim::Body("tibia_l", 3.5598, Vec3(0, -0.20889, 0), Inertia(0.059175, 0.0033172, 0.059175, 0, 0, 0));
	model->addBody(tibia_l);
	//
	talus_l = new OpenSim::Body("talus_l", 0.096016, Vec3(0, 0, 0), Inertia(0.0007392, 0.00070393, 0.00070393, 0, 0, 0));
	model->addBody(talus_l);
	//
	calcn_l = new OpenSim::Body("calcn_l", 1.2002, Vec3(0.083451, 0.026323, 0), Inertia(0.0010349, 0.0026571, 0.0026571, 0, 0, 0));
	model->addBody(calcn_l);
	//
	toes_l = new OpenSim::Body("toes_l", 0.20797, Vec3(0.028874, 0.0052645, 0.015355), Inertia(7.392e-05, 0.00013726, 0.00013726, 0, 0, 0));
	model->addBody(toes_l);
	//
	torso = new OpenSim::Body("torso", 25.7578, Vec3(-0.034919, 0.30957, 0), Inertia(1.4413, 0.91159, 1.4413, 0, 0, 0));
	model->addBody(torso);
	//
	humerus_r = new OpenSim::Body("humerus_r", 1.9515, Vec3(0, -0.19658, 0), Inertia(0.014983, 0.0028561, 0.014983, 0, 0, 0));
	model->addBody(humerus_r);
	//
	ulna_r = new OpenSim::Body("ulna_r", 0.5833, Vec3(0, -0.12876, 0), Inertia(0.0030772, 0.00033941, 0.0030772, 0, 0, 0));
	model->addBody(ulna_r);
	//
	radius_r = new OpenSim::Body("radius_r", 0.5833, Vec3(0, -0.12876, 0), Inertia(0.0030772, 0.00033941, 0.0030772, 0, 0, 0));
	model->addBody(radius_r);
	//
	hand_r = new OpenSim::Body("hand_r", 0.43927, Vec3(0, -0.069811, 0), Inertia(0.00090017, 0.00055201, 0.0013523, 0, 0, 0));
	model->addBody(hand_r);
	//
	humerus_l = new OpenSim::Body("humerus_l", 1.9515, Vec3(0, -0.19658, 0), Inertia(0.014983, 0.0028561, 0.014983, 0, 0, 0));
	model->addBody(humerus_l);
	//
	ulna_l = new OpenSim::Body("ulna_l", 0.5833, Vec3(0, -0.12876, 0), Inertia(0.0030772, 0.00033941, 0.0030772, 0, 0, 0));
	model->addBody(ulna_l);
	//
	radius_l = new OpenSim::Body("radius_l", 0.5833, Vec3(0, -0.12876, 0), Inertia(0.0030772, 0.00033941, 0.0030772, 0, 0, 0));
	model->addBody(radius_l);
	//
	hand_l = new OpenSim::Body("hand_l", 0.43927, Vec3(0, -0.069811, 0), Inertia(0.00090017, 0.00055201, 0.0013523, 0, 0, 0));
	model->addBody(hand_l);
	//
	//
	SpatialTransform st_pelvis_ground_pelvis;
	st_pelvis_ground_pelvis[0].setCoordinateNames(OpenSim::Array<std::string>("pelvis_tilt", 1, 1));
	st_pelvis_ground_pelvis[0].setFunction(new LinearFunction());
	st_pelvis_ground_pelvis[0].setAxis(Vec3(0, 0, 1));
	st_pelvis_ground_pelvis[1].setCoordinateNames(OpenSim::Array<std::string>("pelvis_list", 1, 1));
	st_pelvis_ground_pelvis[1].setFunction(new LinearFunction());
	st_pelvis_ground_pelvis[1].setAxis(Vec3(1, 0, 0));
	st_pelvis_ground_pelvis[2].setCoordinateNames(OpenSim::Array<std::string>("pelvis_rotation", 1, 1));
	st_pelvis_ground_pelvis[2].setFunction(new LinearFunction());
	st_pelvis_ground_pelvis[2].setAxis(Vec3(0, 1, 0));
	st_pelvis_ground_pelvis[3].setCoordinateNames(OpenSim::Array<std::string>("pelvis_tx", 1, 1));
	st_pelvis_ground_pelvis[3].setFunction(new LinearFunction());
	st_pelvis_ground_pelvis[3].setAxis(Vec3(1, 0, 0));
	st_pelvis_ground_pelvis[4].setCoordinateNames(OpenSim::Array<std::string>("pelvis_ty", 1, 1));
	st_pelvis_ground_pelvis[4].setFunction(new LinearFunction());
	st_pelvis_ground_pelvis[4].setAxis(Vec3(0, 1, 0));
	st_pelvis_ground_pelvis[5].setCoordinateNames(OpenSim::Array<std::string>("pelvis_tz", 1, 1));
	st_pelvis_ground_pelvis[5].setFunction(new LinearFunction());
	st_pelvis_ground_pelvis[5].setAxis(Vec3(0, 0, 1));
	ground_pelvis = new CustomJoint("ground_pelvis", model->getGround(), Vec3(0, 0, 0), Vec3(0, 0, 0), *pelvis, Vec3(0, 0, 0), Vec3(0, 0, 0), st_pelvis_ground_pelvis);
	model->addJoint(ground_pelvis);
	//
	SpatialTransform st_femur_r_hip_r;
	st_femur_r_hip_r[0].setCoordinateNames(OpenSim::Array<std::string>("hip_flexion_r", 1, 1));
	st_femur_r_hip_r[0].setFunction(new LinearFunction());
	st_femur_r_hip_r[0].setAxis(Vec3(0, 0, 1));
	st_femur_r_hip_r[1].setCoordinateNames(OpenSim::Array<std::string>("hip_adduction_r", 1, 1));
	st_femur_r_hip_r[1].setFunction(new LinearFunction());
	st_femur_r_hip_r[1].setAxis(Vec3(1, 0, 0));
	st_femur_r_hip_r[2].setCoordinateNames(OpenSim::Array<std::string>("hip_rotation_r", 1, 1));
	st_femur_r_hip_r[2].setFunction(new LinearFunction());
	st_femur_r_hip_r[2].setAxis(Vec3(0, 1, 0));
	hip_r = new CustomJoint("hip_r", *pelvis, Vec3(-0.062385, -0.058326, 0.07284), Vec3(0, 0, 0), *femur_r, Vec3(0, 0, 0), Vec3(0, 0, 0), st_femur_r_hip_r);
	model->addJoint(hip_r);
	//
	SpatialTransform st_tibia_r_knee_r;
	st_tibia_r_knee_r[2].setCoordinateNames(OpenSim::Array<std::string>("knee_angle_r", 1, 1));
	st_tibia_r_knee_r[2].setFunction(new LinearFunction());
	st_tibia_r_knee_r[2].setAxis(Vec3(0, 0, 1));

	knee_r = new CustomJoint("knee_r", *femur_r, Vec3(0.0041, -0.41, 0), Vec3(0, 0, 0), *tibia_r, Vec3(0, 0, 0), Vec3(0, 0, 0), st_tibia_r_knee_r);
	model->addJoint(knee_r);
	//
	SpatialTransform st_talus_r_ankle_r;
	st_talus_r_ankle_r[0].setCoordinateNames(OpenSim::Array<std::string>("ankle_angle_r", 1, 1));
	st_talus_r_ankle_r[0].setFunction(new LinearFunction());
	st_talus_r_ankle_r[0].setAxis(Vec3(-0.10501, -0.17402, 0.97913));
	ankle_r = new CustomJoint("ankle_r", *tibia_r, Vec3(0, -0.48112, 0), Vec3(0, 0, 0), *talus_r, Vec3(0, 0, 0), Vec3(0, 0, 0), st_talus_r_ankle_r);
	model->addJoint(ankle_r);
	//
	SpatialTransform st_calcn_r_subtalar_r;
	st_calcn_r_subtalar_r[0].setCoordinateNames(OpenSim::Array<std::string>("subtalar_angle_r", 1, 1));
	st_calcn_r_subtalar_r[0].setFunction(new LinearFunction());
	st_calcn_r_subtalar_r[0].setAxis(Vec3(0.78718, 0.60475, -0.12095));
	subtalar_r = new CustomJoint("subtalar_r", *talus_r, Vec3(-0.040699, -0.036808, 0.0069492), Vec3(0, 0, 0), *calcn_r, Vec3(0, 0, 0), Vec3(0, 0, 0), st_calcn_r_subtalar_r);
	model->addJoint(subtalar_r);
	//
	SpatialTransform st_toes_r_mtp_r;
	st_toes_r_mtp_r[0].setCoordinateNames(OpenSim::Array<std::string>("mtp_angle_r", 1, 1));
	st_toes_r_mtp_r[0].setFunction(new LinearFunction());
	st_toes_r_mtp_r[0].setAxis(Vec3(-0.58095, 0, 0.81394));
	mtp_r = new CustomJoint("mtp_r", *calcn_r, Vec3(0.14921, -0.0017548, 0.00094761), Vec3(0, 0, 0), *toes_r, Vec3(0, 0, 0), Vec3(0, 0, 0), st_toes_r_mtp_r);
	model->addJoint(mtp_r);
	//
	SpatialTransform st_femur_l_hip_l;
	st_femur_l_hip_l[0].setCoordinateNames(OpenSim::Array<std::string>("hip_flexion_l", 1, 1));
	st_femur_l_hip_l[0].setFunction(new LinearFunction());
	st_femur_l_hip_l[0].setAxis(Vec3(0, 0, 1));
	st_femur_l_hip_l[1].setCoordinateNames(OpenSim::Array<std::string>("hip_adduction_l", 1, 1));
	st_femur_l_hip_l[1].setFunction(new LinearFunction());
	st_femur_l_hip_l[1].setAxis(Vec3(-1, 0, 0));
	st_femur_l_hip_l[2].setCoordinateNames(OpenSim::Array<std::string>("hip_rotation_l", 1, 1));
	st_femur_l_hip_l[2].setFunction(new LinearFunction());
	st_femur_l_hip_l[2].setAxis(Vec3(0, -1, 0));
	hip_l = new CustomJoint("hip_l", *pelvis, Vec3(-0.062385, -0.058326, -0.07284), Vec3(0, 0, 0), *femur_l, Vec3(0, 0, 0), Vec3(0, 0, 0), st_femur_l_hip_l);
	model->addJoint(hip_l);
	//
	SpatialTransform st_tibia_l_knee_l;
	st_tibia_l_knee_l[2].setCoordinateNames(OpenSim::Array<std::string>("knee_angle_l", 1, 1));
	st_tibia_l_knee_l[2].setFunction(new LinearFunction());
	st_tibia_l_knee_l[2].setAxis(Vec3(0, 0, 1));
	knee_l = new CustomJoint("knee_l", *femur_l, Vec3(0.0041, -0.41, 0), Vec3(0, 0, 0), *tibia_l, Vec3(0, 0, 0), Vec3(0, 0, 0), st_tibia_l_knee_l);
	model->addJoint(knee_l);
	//
	SpatialTransform st_talus_l_ankle_l;
	st_talus_l_ankle_l[0].setCoordinateNames(OpenSim::Array<std::string>("ankle_angle_l", 1, 1));
	st_talus_l_ankle_l[0].setFunction(new LinearFunction());
	st_talus_l_ankle_l[0].setAxis(Vec3(0.10501, 0.17402, 0.97913));
	ankle_l = new CustomJoint("ankle_l", *tibia_l, Vec3(0, -0.48112, 0), Vec3(0, 0, 0), *talus_l, Vec3(0, 0, 0), Vec3(0, 0, 0), st_talus_l_ankle_l);
	model->addJoint(ankle_l);
	//
	SpatialTransform st_calcn_l_subtalar_l;
	st_calcn_l_subtalar_l[0].setCoordinateNames(OpenSim::Array<std::string>("subtalar_angle_l", 1, 1));
	st_calcn_l_subtalar_l[0].setFunction(new LinearFunction());
	st_calcn_l_subtalar_l[0].setAxis(Vec3(-0.78718, -0.60475, -0.12095));
	subtalar_l = new CustomJoint("subtalar_l", *talus_l, Vec3(-0.040699, -0.036808, -0.0069492), Vec3(0, 0, 0), *calcn_l, Vec3(0, 0, 0), Vec3(0, 0, 0), st_calcn_l_subtalar_l);
	model->addJoint(subtalar_l);
	//
	SpatialTransform st_toes_l_mtp_l;
	st_toes_l_mtp_l[0].setCoordinateNames(OpenSim::Array<std::string>("mtp_angle_l", 1, 1));
	st_toes_l_mtp_l[0].setFunction(new LinearFunction());
	st_toes_l_mtp_l[0].setAxis(Vec3(0.58095, 0, 0.81394));
	mtp_l = new CustomJoint("mtp_l", *calcn_l, Vec3(0.14921, -0.0017548, -0.00094761), Vec3(0, 0, 0), *toes_l, Vec3(0, 0, 0), Vec3(0, 0, 0), st_toes_l_mtp_l);
	model->addJoint(mtp_l);
	//
	SpatialTransform st_torso_back;
	st_torso_back[0].setCoordinateNames(OpenSim::Array<std::string>("lumbar_extension", 1, 1));
	st_torso_back[0].setFunction(new LinearFunction());
	st_torso_back[0].setAxis(Vec3(0, 0, 1));
	st_torso_back[1].setCoordinateNames(OpenSim::Array<std::string>("lumbar_bending", 1, 1));
	st_torso_back[1].setFunction(new LinearFunction());
	st_torso_back[1].setAxis(Vec3(1, 0, 0));
	st_torso_back[2].setCoordinateNames(OpenSim::Array<std::string>("lumbar_rotation", 1, 1));
	st_torso_back[2].setFunction(new LinearFunction());
	st_torso_back[2].setAxis(Vec3(0, 1, 0));
	back = new CustomJoint("back", *pelvis, Vec3(-0.088857, 0.071915, 0), Vec3(0, 0, 0), *torso, Vec3(0, 0, 0), Vec3(0, 0, 0), st_torso_back);
	model->addJoint(back);
	//
	SpatialTransform st_humerus_r_acromial_r;
	st_humerus_r_acromial_r[0].setCoordinateNames(OpenSim::Array<std::string>("arm_flex_r", 1, 1));
	st_humerus_r_acromial_r[0].setFunction(new LinearFunction());
	st_humerus_r_acromial_r[0].setAxis(Vec3(0, 0, 1));
	st_humerus_r_acromial_r[1].setCoordinateNames(OpenSim::Array<std::string>("arm_add_r", 1, 1));
	st_humerus_r_acromial_r[1].setFunction(new LinearFunction());
	st_humerus_r_acromial_r[1].setAxis(Vec3(1, 0, 0));
	st_humerus_r_acromial_r[2].setCoordinateNames(OpenSim::Array<std::string>("arm_rot_r", 1, 1));
	st_humerus_r_acromial_r[2].setFunction(new LinearFunction());
	st_humerus_r_acromial_r[2].setAxis(Vec3(0, 1, 0));
	acromial_r = new CustomJoint("acromial_r", *torso, Vec3(0.0036723, 0.35939, 0.18354), Vec3(0, 0, 0), *humerus_r, Vec3(0, 0, 0), Vec3(0, 0, 0), st_humerus_r_acromial_r);
	model->addJoint(acromial_r);
	//
	SpatialTransform st_ulna_r_elbow_r;
	st_ulna_r_elbow_r[0].setCoordinateNames(OpenSim::Array<std::string>("elbow_flex_r", 1, 1));
	st_ulna_r_elbow_r[0].setFunction(new LinearFunction());
	st_ulna_r_elbow_r[0].setAxis(Vec3(0.22605, 0.022269, 0.97386));
	elbow_r = new CustomJoint("elbow_r", *humerus_r, Vec3(0.011167, -0.3421, -0.0081518), Vec3(0, 0, 0), *ulna_r, Vec3(0, 0, 0), Vec3(0, 0, 0), st_ulna_r_elbow_r);
	model->addJoint(elbow_r);
	//
	SpatialTransform st_radius_r_radioulnar_r;
	st_radius_r_radioulnar_r[0].setCoordinateNames(OpenSim::Array<std::string>("pro_sup_r", 1, 1));
	st_radius_r_radioulnar_r[0].setFunction(new LinearFunction());
	st_radius_r_radioulnar_r[0].setAxis(Vec3(0.056398, 0.99841, 0.001952));
	radioulnar_r = new CustomJoint("radioulnar_r", *ulna_r, Vec3(-0.0050876, -0.013896, 0.019727), Vec3(0, 0, 0), *radius_r, Vec3(0, 0, 0), Vec3(0, 0, 0), st_radius_r_radioulnar_r);
	model->addJoint(radioulnar_r);
	//
	SpatialTransform st_hand_r_radius_hand_r;
	st_hand_r_radius_hand_r[0].setCoordinateNames(OpenSim::Array<std::string>("wrist_flex_r", 1, 1));
	st_hand_r_radius_hand_r[0].setFunction(new LinearFunction());
	st_hand_r_radius_hand_r[0].setAxis(Vec3(0, 0, 1));
	st_hand_r_radius_hand_r[1].setCoordinateNames(OpenSim::Array<std::string>("wrist_dev_r", 1, 1));
	st_hand_r_radius_hand_r[1].setFunction(new LinearFunction());
	st_hand_r_radius_hand_r[1].setAxis(Vec3(1, 0, 0));
	//st_hand_r_radius_hand_r[2].setCoordinateNames(OpenSim::Array<std::string>(" ", 1, 1));
	//st_hand_r_radius_hand_r[2].setFunction(new Constant(0));
	st_hand_r_radius_hand_r[2].setAxis(Vec3(0, 1, 0));
	radius_hand_r = new CustomJoint("radius_hand_r", *radius_r, Vec3(-0.0066532, -0.25197, 0.010293), Vec3(0, 0, 0), *hand_r, Vec3(0, 0, 0), Vec3(0, 0, 0), st_hand_r_radius_hand_r);
	model->addJoint(radius_hand_r);
	//
	SpatialTransform st_humerus_l_acromial_l;
	st_humerus_l_acromial_l[0].setCoordinateNames(OpenSim::Array<std::string>("arm_flex_l", 1, 1));
	st_humerus_l_acromial_l[0].setFunction(new LinearFunction());
	st_humerus_l_acromial_l[0].setAxis(Vec3(0, 0, 1));
	st_humerus_l_acromial_l[1].setCoordinateNames(OpenSim::Array<std::string>("arm_add_l", 1, 1));
	st_humerus_l_acromial_l[1].setFunction(new LinearFunction());
	st_humerus_l_acromial_l[1].setAxis(Vec3(-1, 0, 0));
	st_humerus_l_acromial_l[2].setCoordinateNames(OpenSim::Array<std::string>("arm_rot_l", 1, 1));
	st_humerus_l_acromial_l[2].setFunction(new LinearFunction());
	st_humerus_l_acromial_l[2].setAxis(Vec3(0, -1, 0));
	acromial_l = new CustomJoint("acromial_l", *torso, Vec3(0.0036723, 0.35939, -0.18354), Vec3(0, 0, 0), *humerus_l, Vec3(0, 0, 0), Vec3(0, 0, 0), st_humerus_l_acromial_l);
	model->addJoint(acromial_l);
	//
	SpatialTransform st_ulna_l_elbow_l;
	st_ulna_l_elbow_l[0].setCoordinateNames(OpenSim::Array<std::string>("elbow_flex_l", 1, 1));
	st_ulna_l_elbow_l[0].setFunction(new LinearFunction());
	st_ulna_l_elbow_l[0].setAxis(Vec3(-0.22605, -0.022269, 0.97386));
	elbow_l = new CustomJoint("elbow_l", *humerus_l, Vec3(0.011167, -0.3421, 0.0081518), Vec3(0, 0, 0), *ulna_l, Vec3(0, 0, 0), Vec3(0, 0, 0), st_ulna_l_elbow_l);
	model->addJoint(elbow_l);
	//
	SpatialTransform st_radius_l_radioulnar_l;
	st_radius_l_radioulnar_l[0].setCoordinateNames(OpenSim::Array<std::string>("pro_sup_l", 1, 1));
	st_radius_l_radioulnar_l[0].setFunction(new LinearFunction());
	st_radius_l_radioulnar_l[0].setAxis(Vec3(-0.056398, -0.99841, 0.001952));
	radioulnar_l = new CustomJoint("radioulnar_l", *ulna_l, Vec3(-0.0050876, -0.013896, -0.019727), Vec3(0, 0, 0), *radius_l, Vec3(0, 0, 0), Vec3(0, 0, 0), st_radius_l_radioulnar_l);
	model->addJoint(radioulnar_l);
	//
	SpatialTransform st_hand_l_radius_hand_l;
	st_hand_l_radius_hand_l[0].setCoordinateNames(OpenSim::Array<std::string>("wrist_flex_l", 1, 1));
	st_hand_l_radius_hand_l[0].setFunction(new LinearFunction());
	st_hand_l_radius_hand_l[0].setAxis(Vec3(0, 0, 1));
	st_hand_l_radius_hand_l[1].setCoordinateNames(OpenSim::Array<std::string>("wrist_dev_l", 1, 1));
	st_hand_l_radius_hand_l[1].setFunction(new LinearFunction());
	st_hand_l_radius_hand_l[1].setAxis(Vec3(-1, 0, 0));
	//st_hand_l_radius_hand_l[2].setCoordinateNames(OpenSim::Array<std::string>(" ", 1, 1));
	//st_hand_l_radius_hand_l[2].setFunction(new Constant(0));
	st_hand_l_radius_hand_l[2].setAxis(Vec3(0, -1, 0));
	radius_hand_l = new CustomJoint("radius_hand_l", *radius_l, Vec3(-0.0066532, -0.25197, -0.010293), Vec3(0, 0, 0), *hand_l, Vec3(0, 0, 0), Vec3(0, 0, 0), st_hand_l_radius_hand_l);
	model->addJoint(radius_hand_l);

	Vector QsUs(NX);

	// Assign inputs to model variables
	for (int i = 0; i < NX; ++i) QsUs[i] = x[i];    // states
	for (int i = 0; i < NDOF; ++i) ua[i] = u[i];    // acceleration controls
	//for (int i = 0; i < 36; ++i) ugrf[i] = u[NDOF + i]; // grf controls (right then left)
	for (int i = 0; i < 24; ++i) contactPrms[i] = p[i]; // contact parameters



	osim_double_adouble radius = 0.02;
	osim_double_adouble sFri = p[20];
	osim_double_adouble dFri = p[21];
	osim_double_adouble vFri = p[22];
	osim_double_adouble tV = p[23];
	Vec3 normal(0, 1, 0);
	osim_double_adouble offset = 0;

	Vec3 rSphPos1 = Vec3(p[0], p[1], p[2]);
	Vec3 rSphPos2 = Vec3(p[3], p[4], p[5]);
	Vec3 rSphPos3 = Vec3(p[6], p[7], p[8]);
	Vec3 rSphPos4 = Vec3(p[9], p[10], p[11]);
	Vec3 rSphPos5 = Vec3(p[12], p[13], p[14]);
	Vec3 rSphPos6 = Vec3(p[15], p[16], p[17]);
	Vec3 rSphPos7 = Vec3(0, rSphPos6[1], 0);

	Vec3 lSphPos1 = Vec3(p[0], p[1], -p[2]);
	Vec3 lSphPos2 = Vec3(p[3], p[4], -p[5]);
	Vec3 lSphPos3 = Vec3(p[6], p[7], -p[8]);
	Vec3 lSphPos4 = Vec3(p[9], p[10], -p[11]);
	Vec3 lSphPos5 = Vec3(p[12], p[13], -p[14]);
	Vec3 lSphPos6 = Vec3(p[15], p[16], -p[17]);
	Vec3 lSphPos7 = Vec3(0, lSphPos6[1], 0);

	osim_double_adouble stiffness = p[18];
	osim_double_adouble dissipation = p[19];

	//OpenSim::Body toes_r = model->updBodySet().get("toes_r");
	//OpenSim::Body calcn_r = model->updBodySet().get("calcn_r");

	HuntCrossleyForce_smooth* rsph1 = new HuntCrossleyForce_smooth("rpsh1", "toes_r", rSphPos1, radius, stiffness, dissipation, sFri, dFri, vFri, tV, normal, offset);
	model->addComponent(rsph1);
	rsph1->connectSocket_body_sphere(*toes_r);
	HuntCrossleyForce_smooth* rsph2 = new HuntCrossleyForce_smooth("rpsh2", "toes_r", rSphPos2, radius, stiffness, dissipation, sFri, dFri, vFri, tV, normal, offset);
	model->addComponent(rsph2);
	rsph2->connectSocket_body_sphere(*toes_r);
	HuntCrossleyForce_smooth* rsph3 = new HuntCrossleyForce_smooth("rpsh3", "calcn_r", rSphPos3, radius, stiffness, dissipation, sFri, dFri, vFri, tV, normal, offset);
	model->addComponent(rsph3);
	rsph3->connectSocket_body_sphere(*calcn_r);
	HuntCrossleyForce_smooth* rsph4 = new HuntCrossleyForce_smooth("rpsh4", "calcn_r", rSphPos4, radius, stiffness, dissipation, sFri, dFri, vFri, tV, normal, offset);
	model->addComponent(rsph4);
	rsph4->connectSocket_body_sphere(*calcn_r);
	HuntCrossleyForce_smooth* rsph5 = new HuntCrossleyForce_smooth("rpsh5", "calcn_r", rSphPos5, radius, stiffness, dissipation, sFri, dFri, vFri, tV, normal, offset);
	model->addComponent(rsph5);
	rsph5->connectSocket_body_sphere(*calcn_r);
	HuntCrossleyForce_smooth* rsph6 = new HuntCrossleyForce_smooth("rpsh6", "calcn_r", rSphPos6, radius, stiffness, dissipation, sFri, dFri, vFri, tV, normal, offset);
	model->addComponent(rsph6);
	rsph6->connectSocket_body_sphere(*calcn_r);
	HuntCrossleyForce_smooth* rsph7 = new HuntCrossleyForce_smooth("rpsh7", "calcn_r", rSphPos7, radius, stiffness, dissipation, sFri, dFri, vFri, tV, normal, offset);
	model->addComponent(rsph7);
	rsph7->connectSocket_body_sphere(*calcn_r);

	HuntCrossleyForce_smooth* lsph1 = new HuntCrossleyForce_smooth("lpsh1", "toes_l", lSphPos1, radius, stiffness, dissipation, sFri, dFri, vFri, tV, normal, offset);
	model->addComponent(lsph1);
	lsph1->connectSocket_body_sphere(*toes_l);
	HuntCrossleyForce_smooth* lsph2 = new HuntCrossleyForce_smooth("lpsh2", "toes_l", lSphPos2, radius, stiffness, dissipation, sFri, dFri, vFri, tV, normal, offset);
	model->addComponent(lsph2);
	lsph2->connectSocket_body_sphere(*toes_l);
	HuntCrossleyForce_smooth* lsph3 = new HuntCrossleyForce_smooth("lpsh3", "calcn_l", lSphPos3, radius, stiffness, dissipation, sFri, dFri, vFri, tV, normal, offset);
	model->addComponent(lsph3);
	lsph3->connectSocket_body_sphere(*calcn_l);
	HuntCrossleyForce_smooth* lsph4 = new HuntCrossleyForce_smooth("lpsh4", "calcn_l", lSphPos4, radius, stiffness, dissipation, sFri, dFri, vFri, tV, normal, offset);
	model->addComponent(lsph4);
	lsph4->connectSocket_body_sphere(*calcn_l);
	HuntCrossleyForce_smooth* lsph5 = new HuntCrossleyForce_smooth("lpsh5", "calcn_l", lSphPos5, radius, stiffness, dissipation, sFri, dFri, vFri, tV, normal, offset);
	model->addComponent(lsph5);
	lsph5->connectSocket_body_sphere(*calcn_l);
	HuntCrossleyForce_smooth* lsph6 = new HuntCrossleyForce_smooth("lpsh6", "calcn_l", lSphPos6, radius, stiffness, dissipation, sFri, dFri, vFri, tV, normal, offset);
	model->addComponent(lsph6);
	lsph6->connectSocket_body_sphere(*calcn_l);
	HuntCrossleyForce_smooth* lsph7 = new HuntCrossleyForce_smooth("lpsh7", "calcn_l", lSphPos7, radius, stiffness, dissipation, sFri, dFri, vFri, tV, normal, offset);
	model->addComponent(lsph7);
	lsph7->connectSocket_body_sphere(*calcn_l);

	// Initialize the system and state.
	State* state = new State(model->initSystem());



	//Vector QsUs(NX);

	// Assign inputs to model variables
	//for (int i = 0; i < NX; ++i) QsUs[i] = x[i];    // states
	//for (int i = 0; i < NDOF; ++i) ua[i] = u[i];    // acceleration controls
	//for (int i = 0; i < 18; ++i) ugrf[i] = u[NDOF + i]; // grf controls

	model->setStateVariableValues(*state, QsUs);
	model->realizeVelocity(*state);

	// Get system's COM Position & Velocity
	Vec3 COM_p_G = model->calcMassCenterPosition(*state);
	Vec3 COM_v_G = model->calcMassCenterVelocity(*state);

	// Angular momentum calculations
	int femur_r_ind = model->getBodySet().get("femur_r").getMobilizedBodyIndex();
	Mat33 femur_r_I = femur_r->getInertia().toMat33();
	Mat33 femur_r_R = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(femur_r_ind)).getBodyRotation(*state);
	Mat33 femur_r_I_G = femur_r_R * femur_r_I;
	Vec3 femur_r_com = femur_r->getMassCenter();
	Vec3 femur_r_com_G = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(femur_r_ind)).findStationLocationInGround(*state, femur_r_com);
	Vec3 COM_femur_r_G = femur_r_com_G - COM_p_G;
	Vec3 femur_r_com_v_G = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(femur_r_ind)).findStationVelocityInGround(*state, femur_r_com);
	Vec3 COM_femur_r_v_G = femur_r_com_v_G - COM_v_G;
	osim_double_adouble femur_r_mass = femur_r->getMass();
	Vec3 femur_r_w = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(femur_r_ind)).getBodyVelocity(*state)[0];
	Vec3 femur_r_H_COM = crossproduct(COM_femur_r_G, COM_femur_r_v_G*femur_r_mass) + femur_r_I_G * femur_r_w;

	int femur_l_ind = model->getBodySet().get("femur_l").getMobilizedBodyIndex();
	Mat33 femur_l_I = femur_l->getInertia().toMat33();
	Mat33 femur_l_R = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(femur_l_ind)).getBodyRotation(*state);
	Mat33 femur_l_I_G = femur_l_R * femur_l_I;
	Vec3 femur_l_com = femur_l->getMassCenter();
	Vec3 femur_l_com_G = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(femur_l_ind)).findStationLocationInGround(*state, femur_l_com);
	Vec3 COM_femur_l_G = femur_l_com_G - COM_p_G;
	Vec3 femur_l_com_v_G = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(femur_l_ind)).findStationVelocityInGround(*state, femur_l_com);
	Vec3 COM_femur_l_v_G = femur_l_com_v_G - COM_v_G;
	osim_double_adouble femur_l_mass = femur_l->getMass();
	Vec3 femur_l_w = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(femur_l_ind)).getBodyVelocity(*state)[0];
	Vec3 femur_l_H_COM = crossproduct(COM_femur_l_G, COM_femur_l_v_G*femur_l_mass) + femur_l_I_G * femur_l_w;

	int tibia_r_ind = model->getBodySet().get("tibia_r").getMobilizedBodyIndex();
	Mat33 tibia_r_I = tibia_r->getInertia().toMat33();
	Mat33 tibia_r_R = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(tibia_r_ind)).getBodyRotation(*state);
	Mat33 tibia_r_I_G = tibia_r_R * tibia_r_I;
	Vec3 tibia_r_com = tibia_r->getMassCenter();
	Vec3 tibia_r_com_G = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(tibia_r_ind)).findStationLocationInGround(*state, tibia_r_com);
	Vec3 COM_tibia_r_G = tibia_r_com_G - COM_p_G;
	Vec3 tibia_r_com_v_G = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(tibia_r_ind)).findStationVelocityInGround(*state, tibia_r_com);
	Vec3 COM_tibia_r_v_G = tibia_r_com_v_G - COM_v_G;
	osim_double_adouble tibia_r_mass = tibia_r->getMass();
	Vec3 tibia_r_w = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(tibia_r_ind)).getBodyVelocity(*state)[0];
	Vec3 tibia_r_H_COM = crossproduct(COM_tibia_r_G, COM_tibia_r_v_G*tibia_r_mass) + tibia_r_I_G * tibia_r_w;

	int tibia_l_ind = model->getBodySet().get("tibia_l").getMobilizedBodyIndex();
	Mat33 tibia_l_I = tibia_l->getInertia().toMat33();
	Mat33 tibia_l_R = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(tibia_l_ind)).getBodyRotation(*state);
	Mat33 tibia_l_I_G = tibia_l_R * tibia_l_I;
	Vec3 tibia_l_com = tibia_l->getMassCenter();
	Vec3 tibia_l_com_G = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(tibia_l_ind)).findStationLocationInGround(*state, tibia_l_com);
	Vec3 COM_tibia_l_G = tibia_l_com_G - COM_p_G;
	Vec3 tibia_l_com_v_G = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(tibia_l_ind)).findStationVelocityInGround(*state, tibia_l_com);
	Vec3 COM_tibia_l_v_G = tibia_l_com_v_G - COM_v_G;
	osim_double_adouble tibia_l_mass = tibia_l->getMass();
	Vec3 tibia_l_w = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(tibia_l_ind)).getBodyVelocity(*state)[0];
	Vec3 tibia_l_H_COM = crossproduct(COM_tibia_l_G, COM_tibia_l_v_G*tibia_l_mass) + tibia_l_I_G * tibia_l_w;

	int talus_r_ind = model->getBodySet().get("talus_r").getMobilizedBodyIndex();
	Mat33 talus_r_I = talus_r->getInertia().toMat33();
	Mat33 talus_r_R = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(talus_r_ind)).getBodyRotation(*state);
	Mat33 talus_r_I_G = talus_r_R * talus_r_I;
	Vec3 talus_r_com = talus_r->getMassCenter();
	Vec3 talus_r_com_G = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(talus_r_ind)).findStationLocationInGround(*state, talus_r_com);
	Vec3 COM_talus_r_G = talus_r_com_G - COM_p_G;
	Vec3 talus_r_com_v_G = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(talus_r_ind)).findStationVelocityInGround(*state, talus_r_com);
	Vec3 COM_talus_r_v_G = talus_r_com_v_G - COM_v_G;
	osim_double_adouble talus_r_mass = talus_r->getMass();
	Vec3 talus_r_w = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(talus_r_ind)).getBodyVelocity(*state)[0];
	Vec3 talus_r_H_COM = crossproduct(COM_talus_r_G, COM_talus_r_v_G*talus_r_mass) + talus_r_I_G * talus_r_w;

	int talus_l_ind = model->getBodySet().get("talus_l").getMobilizedBodyIndex();
	Mat33 talus_l_I = talus_l->getInertia().toMat33();
	Mat33 talus_l_R = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(talus_l_ind)).getBodyRotation(*state);
	Mat33 talus_l_I_G = talus_l_R * talus_l_I;
	Vec3 talus_l_com = talus_l->getMassCenter();
	Vec3 talus_l_com_G = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(talus_l_ind)).findStationLocationInGround(*state, talus_l_com);
	Vec3 COM_talus_l_G = talus_l_com_G - COM_p_G;
	Vec3 talus_l_com_v_G = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(talus_l_ind)).findStationVelocityInGround(*state, talus_l_com);
	Vec3 COM_talus_l_v_G = talus_l_com_v_G - COM_v_G;
	osim_double_adouble talus_l_mass = talus_l->getMass();
	Vec3 talus_l_w = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(talus_l_ind)).getBodyVelocity(*state)[0];
	Vec3 talus_l_H_COM = crossproduct(COM_talus_l_G, COM_talus_l_v_G*talus_l_mass) + talus_l_I_G * talus_l_w;

	int calcn_r_ind = model->getBodySet().get("calcn_r").getMobilizedBodyIndex();
	Mat33 calcn_r_I = calcn_r->getInertia().toMat33();
	Mat33 calcn_r_R = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(calcn_r_ind)).getBodyRotation(*state);
	Mat33 calcn_r_I_G = calcn_r_R * calcn_r_I;
	Vec3 calcn_r_com = calcn_r->getMassCenter();
	Vec3 calcn_r_com_G = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(calcn_r_ind)).findStationLocationInGround(*state, calcn_r_com);
	Vec3 COM_calcn_r_G = calcn_r_com_G - COM_p_G;
	Vec3 calcn_r_com_v_G = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(calcn_r_ind)).findStationVelocityInGround(*state, calcn_r_com);
	Vec3 COM_calcn_r_v_G = calcn_r_com_v_G - COM_v_G;
	osim_double_adouble calcn_r_mass = calcn_r->getMass();
	Vec3 calcn_r_w = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(calcn_r_ind)).getBodyVelocity(*state)[0];
	Vec3 calcn_r_H_COM = crossproduct(COM_calcn_r_G, COM_calcn_r_v_G*calcn_r_mass) + calcn_r_I_G * calcn_r_w;

	int calcn_l_ind = model->getBodySet().get("calcn_l").getMobilizedBodyIndex();
	Mat33 calcn_l_I = calcn_l->getInertia().toMat33();
	Mat33 calcn_l_R = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(calcn_l_ind)).getBodyRotation(*state);
	Mat33 calcn_l_I_G = calcn_l_R * calcn_l_I;
	Vec3 calcn_l_com = calcn_l->getMassCenter();
	Vec3 calcn_l_com_G = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(calcn_l_ind)).findStationLocationInGround(*state, calcn_l_com);
	Vec3 COM_calcn_l_G = calcn_l_com_G - COM_p_G;
	Vec3 calcn_l_com_v_G = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(calcn_l_ind)).findStationVelocityInGround(*state, calcn_l_com);
	Vec3 COM_calcn_l_v_G = calcn_l_com_v_G - COM_v_G;
	osim_double_adouble calcn_l_mass = calcn_l->getMass();
	Vec3 calcn_l_w = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(calcn_l_ind)).getBodyVelocity(*state)[0];
	Vec3 calcn_l_H_COM = crossproduct(COM_calcn_l_G, COM_calcn_l_v_G*calcn_l_mass) + calcn_l_I_G * calcn_l_w;

	int toes_r_ind = model->getBodySet().get("toes_r").getMobilizedBodyIndex();
	Mat33 toes_r_I = toes_r->getInertia().toMat33();
	Mat33 toes_r_R = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(toes_r_ind)).getBodyRotation(*state);
	Mat33 toes_r_I_G = toes_r_R * toes_r_I;
	Vec3 toes_r_com = toes_r->getMassCenter();
	Vec3 toes_r_com_G = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(toes_r_ind)).findStationLocationInGround(*state, toes_r_com);
	Vec3 COM_toes_r_G = toes_r_com_G - COM_p_G;
	Vec3 toes_r_com_v_G = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(toes_r_ind)).findStationVelocityInGround(*state, toes_r_com);
	Vec3 COM_toes_r_v_G = toes_r_com_v_G - COM_v_G;
	osim_double_adouble toes_r_mass = toes_r->getMass();
	Vec3 toes_r_w = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(toes_r_ind)).getBodyVelocity(*state)[0];
	Vec3 toes_r_H_COM = crossproduct(COM_toes_r_G, COM_toes_r_v_G*toes_r_mass) + toes_r_I_G * toes_r_w;

	int toes_l_ind = model->getBodySet().get("toes_l").getMobilizedBodyIndex();
	Mat33 toes_l_I = toes_l->getInertia().toMat33();
	Mat33 toes_l_R = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(toes_l_ind)).getBodyRotation(*state);
	Mat33 toes_l_I_G = toes_l_R * toes_l_I;
	Vec3 toes_l_com = toes_l->getMassCenter();
	Vec3 toes_l_com_G = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(toes_l_ind)).findStationLocationInGround(*state, toes_l_com);
	Vec3 COM_toes_l_G = toes_l_com_G - COM_p_G;
	Vec3 toes_l_com_v_G = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(toes_l_ind)).findStationVelocityInGround(*state, toes_l_com);
	Vec3 COM_toes_l_v_G = toes_l_com_v_G - COM_v_G;
	osim_double_adouble toes_l_mass = toes_l->getMass();
	Vec3 toes_l_w = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(toes_l_ind)).getBodyVelocity(*state)[0];
	Vec3 toes_l_H_COM = crossproduct(COM_toes_l_G, COM_toes_l_v_G*toes_l_mass) + toes_l_I_G * toes_l_w;

	int humerus_r_ind = model->getBodySet().get("humerus_r").getMobilizedBodyIndex();
	Mat33 humerus_r_I = humerus_r->getInertia().toMat33();
	Mat33 humerus_r_R = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(humerus_r_ind)).getBodyRotation(*state);
	Mat33 humerus_r_I_G = humerus_r_R * humerus_r_I;
	Vec3 humerus_r_com = humerus_r->getMassCenter();
	Vec3 humerus_r_com_G = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(humerus_r_ind)).findStationLocationInGround(*state, humerus_r_com);
	Vec3 COM_humerus_r_G = humerus_r_com_G - COM_p_G;
	Vec3 humerus_r_com_v_G = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(humerus_r_ind)).findStationVelocityInGround(*state, humerus_r_com);
	Vec3 COM_humerus_r_v_G = humerus_r_com_v_G - COM_v_G;
	osim_double_adouble humerus_r_mass = humerus_r->getMass();
	Vec3 humerus_r_w = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(humerus_r_ind)).getBodyVelocity(*state)[0];
	Vec3 humerus_r_H_COM = crossproduct(COM_humerus_r_G, COM_humerus_r_v_G*humerus_r_mass) + humerus_r_I_G * humerus_r_w;

	int humerus_l_ind = model->getBodySet().get("humerus_l").getMobilizedBodyIndex();
	Mat33 humerus_l_I = humerus_l->getInertia().toMat33();
	Mat33 humerus_l_R = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(humerus_l_ind)).getBodyRotation(*state);
	Mat33 humerus_l_I_G = humerus_l_R * humerus_l_I;
	Vec3 humerus_l_com = humerus_l->getMassCenter();
	Vec3 humerus_l_com_G = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(humerus_l_ind)).findStationLocationInGround(*state, humerus_l_com);
	Vec3 COM_humerus_l_G = humerus_l_com_G - COM_p_G;
	Vec3 humerus_l_com_v_G = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(humerus_l_ind)).findStationVelocityInGround(*state, humerus_l_com);
	Vec3 COM_humerus_l_v_G = humerus_l_com_v_G - COM_v_G;
	osim_double_adouble humerus_l_mass = humerus_l->getMass();
	Vec3 humerus_l_w = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(humerus_l_ind)).getBodyVelocity(*state)[0];
	Vec3 humerus_l_H_COM = crossproduct(COM_humerus_l_G, COM_humerus_l_v_G*humerus_l_mass) + humerus_l_I_G * humerus_l_w;

	int ulna_r_ind = model->getBodySet().get("ulna_r").getMobilizedBodyIndex();
	Mat33 ulna_r_I = ulna_r->getInertia().toMat33();
	Mat33 ulna_r_R = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(ulna_r_ind)).getBodyRotation(*state);
	Mat33 ulna_r_I_G = ulna_r_R * ulna_r_I;
	Vec3 ulna_r_com = ulna_r->getMassCenter();
	Vec3 ulna_r_com_G = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(ulna_r_ind)).findStationLocationInGround(*state, ulna_r_com);
	Vec3 COM_ulna_r_G = ulna_r_com_G - COM_p_G;
	Vec3 ulna_r_com_v_G = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(ulna_r_ind)).findStationVelocityInGround(*state, ulna_r_com);
	Vec3 COM_ulna_r_v_G = ulna_r_com_v_G - COM_v_G;
	osim_double_adouble ulna_r_mass = ulna_r->getMass();
	Vec3 ulna_r_w = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(ulna_r_ind)).getBodyVelocity(*state)[0];
	Vec3 ulna_r_H_COM = crossproduct(COM_ulna_r_G, COM_ulna_r_v_G*ulna_r_mass) + ulna_r_I_G * ulna_r_w;

	int ulna_l_ind = model->getBodySet().get("ulna_l").getMobilizedBodyIndex();
	Mat33 ulna_l_I = ulna_l->getInertia().toMat33();
	Mat33 ulna_l_R = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(ulna_l_ind)).getBodyRotation(*state);
	Mat33 ulna_l_I_G = ulna_l_R * ulna_l_I;
	Vec3 ulna_l_com = ulna_l->getMassCenter();
	Vec3 ulna_l_com_G = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(ulna_l_ind)).findStationLocationInGround(*state, ulna_l_com);
	Vec3 COM_ulna_l_G = ulna_l_com_G - COM_p_G;
	Vec3 ulna_l_com_v_G = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(ulna_l_ind)).findStationVelocityInGround(*state, ulna_l_com);
	Vec3 COM_ulna_l_v_G = ulna_l_com_v_G - COM_v_G;
	osim_double_adouble ulna_l_mass = ulna_l->getMass();
	Vec3 ulna_l_w = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(ulna_l_ind)).getBodyVelocity(*state)[0];
	Vec3 ulna_l_H_COM = crossproduct(COM_ulna_l_G, COM_ulna_l_v_G*ulna_l_mass) + ulna_l_I_G * ulna_l_w;

	int radius_r_ind = model->getBodySet().get("radius_r").getMobilizedBodyIndex();
	Mat33 radius_r_I = radius_r->getInertia().toMat33();
	Mat33 radius_r_R = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(radius_r_ind)).getBodyRotation(*state);
	Mat33 radius_r_I_G = radius_r_R * radius_r_I;
	Vec3 radius_r_com = radius_r->getMassCenter();
	Vec3 radius_r_com_G = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(radius_r_ind)).findStationLocationInGround(*state, radius_r_com);
	Vec3 COM_radius_r_G = radius_r_com_G - COM_p_G;
	Vec3 radius_r_com_v_G = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(radius_r_ind)).findStationVelocityInGround(*state, radius_r_com);
	Vec3 COM_radius_r_v_G = radius_r_com_v_G - COM_v_G;
	osim_double_adouble radius_r_mass = radius_r->getMass();
	Vec3 radius_r_w = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(radius_r_ind)).getBodyVelocity(*state)[0];
	Vec3 radius_r_H_COM = crossproduct(COM_radius_r_G, COM_radius_r_v_G*radius_r_mass) + radius_r_I_G * radius_r_w;

	int radius_l_ind = model->getBodySet().get("radius_l").getMobilizedBodyIndex();
	Mat33 radius_l_I = radius_l->getInertia().toMat33();
	Mat33 radius_l_R = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(radius_l_ind)).getBodyRotation(*state);
	Mat33 radius_l_I_G = radius_l_R * radius_l_I;
	Vec3 radius_l_com = radius_l->getMassCenter();
	Vec3 radius_l_com_G = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(radius_l_ind)).findStationLocationInGround(*state, radius_l_com);
	Vec3 COM_radius_l_G = radius_l_com_G - COM_p_G;
	Vec3 radius_l_com_v_G = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(radius_l_ind)).findStationVelocityInGround(*state, radius_l_com);
	Vec3 COM_radius_l_v_G = radius_l_com_v_G - COM_v_G;
	osim_double_adouble radius_l_mass = radius_l->getMass();
	Vec3 radius_l_w = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(radius_l_ind)).getBodyVelocity(*state)[0];
	Vec3 radius_l_H_COM = crossproduct(COM_radius_l_G, COM_radius_l_v_G*radius_l_mass) + radius_l_I_G * radius_l_w;

	int hand_r_ind = model->getBodySet().get("hand_r").getMobilizedBodyIndex();
	Mat33 hand_r_I = hand_r->getInertia().toMat33();
	Mat33 hand_r_R = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(hand_r_ind)).getBodyRotation(*state);
	Mat33 hand_r_I_G = hand_r_R * hand_r_I;
	Vec3 hand_r_com = hand_r->getMassCenter();
	Vec3 hand_r_com_G = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(hand_r_ind)).findStationLocationInGround(*state, hand_r_com);
	Vec3 COM_hand_r_G = hand_r_com_G - COM_p_G;
	Vec3 hand_r_com_v_G = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(hand_r_ind)).findStationVelocityInGround(*state, hand_r_com);
	Vec3 COM_hand_r_v_G = hand_r_com_v_G - COM_v_G;
	osim_double_adouble hand_r_mass = hand_r->getMass();
	Vec3 hand_r_w = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(hand_r_ind)).getBodyVelocity(*state)[0];
	Vec3 hand_r_H_COM = crossproduct(COM_hand_r_G, COM_hand_r_v_G*hand_r_mass) + hand_r_I_G * hand_r_w;

	int hand_l_ind = model->getBodySet().get("hand_l").getMobilizedBodyIndex();
	Mat33 hand_l_I = hand_l->getInertia().toMat33();
	Mat33 hand_l_R = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(hand_l_ind)).getBodyRotation(*state);
	Mat33 hand_l_I_G = hand_l_R * hand_l_I;
	Vec3 hand_l_com = hand_l->getMassCenter();
	Vec3 hand_l_com_G = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(hand_l_ind)).findStationLocationInGround(*state, hand_l_com);
	Vec3 COM_hand_l_G = hand_l_com_G - COM_p_G;
	Vec3 hand_l_com_v_G = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(hand_l_ind)).findStationVelocityInGround(*state, hand_l_com);
	Vec3 COM_hand_l_v_G = hand_l_com_v_G - COM_v_G;
	osim_double_adouble hand_l_mass = hand_l->getMass();
	Vec3 hand_l_w = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(hand_l_ind)).getBodyVelocity(*state)[0];
	Vec3 hand_l_H_COM = crossproduct(COM_hand_l_G, COM_hand_l_v_G*hand_l_mass) + hand_l_I_G * hand_l_w;

	int pelvis_ind = model->getBodySet().get("pelvis").getMobilizedBodyIndex();
	Mat33 pelvis_I = pelvis->getInertia().toMat33();
	Mat33 pelvis_R = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(pelvis_ind)).getBodyRotation(*state);
	Mat33 pelvis_I_G = pelvis_R * pelvis_I;
	Vec3 pelvis_com = pelvis->getMassCenter();
	Vec3 pelvis_com_G = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(pelvis_ind)).findStationLocationInGround(*state, pelvis_com);
	Vec3 COM_pelvis_G = pelvis_com_G - COM_p_G;
	Vec3 pelvis_com_v_G = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(pelvis_ind)).findStationVelocityInGround(*state, pelvis_com);
	Vec3 COM_pelvis_v_G = pelvis_com_v_G - COM_v_G;
	osim_double_adouble pelvis_mass = pelvis->getMass();
	Vec3 pelvis_w = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(pelvis_ind)).getBodyVelocity(*state)[0];
	Vec3 pelvis_H_COM = crossproduct(COM_pelvis_G, COM_pelvis_v_G*pelvis_mass) + pelvis_I_G * pelvis_w;

	int torso_ind = model->getBodySet().get("torso").getMobilizedBodyIndex();
	Mat33 torso_I = torso->getInertia().toMat33();
	Mat33 torso_R = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(torso_ind)).getBodyRotation(*state);
	Mat33 torso_I_G = torso_R * torso_I;
	Vec3 torso_com = torso->getMassCenter();
	Vec3 torso_com_G = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(torso_ind)).findStationLocationInGround(*state, torso_com);
	Vec3 COM_torso_G = torso_com_G - COM_p_G;
	Vec3 torso_com_v_G = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(torso_ind)).findStationVelocityInGround(*state, torso_com);
	Vec3 COM_torso_v_G = torso_com_v_G - COM_v_G;
	osim_double_adouble torso_mass = torso->getMass();
	Vec3 torso_w = model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(torso_ind)).getBodyVelocity(*state)[0];
	Vec3 torso_H_COM = crossproduct(COM_torso_G, COM_torso_v_G*torso_mass) + torso_I_G * torso_w;

	// Get calcaneus COM position & velocity in ground
	//Vec3 calcn_r_pG = calcn_r->getPositionInGround(*state);
	//Vec3 calcn_l_pG = calcn_l->getPositionInGround(*state);
	Vec3 calcn_r_COM = calcn_r->getMassCenter();
	Vec3 calcn_r_pG = calcn_r->findStationLocationInGround(*state, calcn_r_COM);

	Vec3 calcn_l_COM = calcn_l->getMassCenter();
	Vec3 calcn_l_pG = calcn_l->findStationLocationInGround(*state, calcn_l_COM);

	Vec3 femur_r_XYZ = femur_r->getTransformInGround(*state).R().convertRotationToBodyFixedXYZ();
	Vec3 femur_l_XYZ = femur_l->getTransformInGround(*state).R().convertRotationToBodyFixedXYZ();
	Vec3 tibia_r_XYZ = tibia_r->getTransformInGround(*state).R().convertRotationToBodyFixedXYZ();
	Vec3 tibia_l_XYZ = tibia_l->getTransformInGround(*state).R().convertRotationToBodyFixedXYZ();
	Vec3 torso_XYZ = torso->getTransformInGround(*state).R().convertRotationToBodyFixedXYZ();

	//SpatialVec calcn_r_vG = calcn_r->getVelocityInGround(*state); // SpatialVec[0] -> Angular, SpatialVec[1] -> Linear
	//SpatialVec calcn_l_vG = calcn_l->getVelocityInGround(*state);
	Vec3 calcn_r_vG = calcn_r->findStationVelocityInGround(*state, calcn_r_COM);
	Vec3 calcn_l_vG = calcn_l->findStationVelocityInGround(*state, calcn_l_COM);

	// Get toes COM position & velocity in ground
	Vec3 toes_r_COM = toes_r->getMassCenter();
	Vec3 toes_r_pG = toes_r->findStationLocationInGround(*state, toes_r_COM);
	Vec3 toes_l_COM = toes_l->getMassCenter();
	Vec3 toes_l_pG = toes_l->findStationVelocityInGround(*state, toes_l_COM);

	Vec3 toes_r_vG = toes_r->findStationVelocityInGround(*state, toes_r_COM); // SpatialVec[0] -> Angular, SpatialVec[1] -> Linear
	Vec3 toes_l_vG = toes_l->findStationVelocityInGround(*state, toes_l_COM);

	// Get tibia Frame position in ground
	Vec3 tibia_r_pG = tibia_r->getPositionInGround(*state);
	Vec3 tibia_l_pG = tibia_l->getPositionInGround(*state);

	// Get radius Frame position in ground 
	Vec3 radius_r_pG = radius_r->getPositionInGround(*state);
	Vec3 radius_l_pG = radius_l->getPositionInGround(*state);

	// Get hand Frame position in ground
	Vec3 hand_r_pG = hand_r->getPositionInGround(*state);
	Vec3 hand_l_pG = hand_l->getPositionInGround(*state);

	// Get torso Frame position in ground
	Vec3 torso_pG = torso->getPositionInGround(*state);

	// Get pelvis Frame position in ground
	Vec3 pelvis_pG = pelvis->getPositionInGround(*state);

	// Get femur Frame position in ground
	Vec3 femur_r_pG = femur_r->getPositionInGround(*state);
	Vec3 femur_l_pG = femur_l->getPositionInGround(*state);

	// Determine resultant distance between segments of interest 

	// Calcaneus-Calcaneus
	osim_double_adouble calcn_mag = pow((pow(calcn_r_pG[0] - calcn_l_pG[0], 2) + pow(calcn_r_pG[1] - calcn_l_pG[1], 2) + pow(calcn_r_pG[2] - calcn_l_pG[2], 2)), 0.5);

	// Toes-Toes
	osim_double_adouble toes_mag = pow((pow(toes_r_pG[0] - toes_l_pG[0], 2) + pow(toes_r_pG[1] - toes_l_pG[1], 2) + pow(toes_r_pG[2] - toes_l_pG[2], 2)), 0.5);

	// Tibia-Tibia
	osim_double_adouble tibia_mag = pow((pow(tibia_r_pG[0] - tibia_l_pG[0], 2) + pow(tibia_r_pG[1] - tibia_l_pG[1], 2) + pow(tibia_r_pG[2] - tibia_l_pG[2], 2)), 0.5);

	// Radius-Radius
	osim_double_adouble radius_mag = pow((pow(radius_r_pG[0] - radius_l_pG[0], 2) + pow(radius_r_pG[1] - radius_l_pG[1], 2) + pow(radius_r_pG[2] - radius_l_pG[2], 2)), 0.5);

	// Hand-Hand
	osim_double_adouble hand_mag = pow((pow(hand_r_pG[0] - hand_l_pG[0], 2) + pow(hand_r_pG[1] - hand_l_pG[1], 2) + pow(hand_r_pG[2] - hand_l_pG[2], 2)), 0.5);

	// Torso-Radius
	osim_double_adouble torsoRadius_r_mag = pow((pow(radius_r_pG[0] - torso_pG[0], 2) + pow(radius_r_pG[1] - torso_pG[1], 2) + pow(radius_r_pG[2] - torso_pG[2], 2)), 0.5);
	osim_double_adouble torsoRadius_l_mag = pow((pow(radius_l_pG[0] - torso_pG[0], 2) + pow(radius_l_pG[1] - torso_pG[1], 2) + pow(radius_l_pG[2] - torso_pG[2], 2)), 0.5);

	// Pelvis-Radius
	osim_double_adouble pelvisRadius_r_mag = pow((pow(radius_r_pG[0] - pelvis_pG[0], 2) + pow(radius_r_pG[1] - pelvis_pG[1], 2) + pow(radius_r_pG[2] - pelvis_pG[2], 2)), 0.5);
	osim_double_adouble pelvisRadius_l_mag = pow((pow(radius_l_pG[0] - pelvis_pG[0], 2) + pow(radius_l_pG[1] - pelvis_pG[1], 2) + pow(radius_l_pG[2] - pelvis_pG[2], 2)), 0.5);

	// Torso-Hand
	osim_double_adouble torsoHand_r_mag = pow((pow(hand_r_pG[0] - torso_pG[0], 2) + pow(hand_r_pG[1] - torso_pG[1], 2) + pow(hand_r_pG[2] - torso_pG[2], 2)), 0.5);
	osim_double_adouble torsoHand_l_mag = pow((pow(hand_l_pG[0] - torso_pG[0], 2) + pow(hand_l_pG[1] - torso_pG[1], 2) + pow(hand_l_pG[2] - torso_pG[2], 2)), 0.5);

	// Tibia-Calcaneus
	osim_double_adouble tibiaR_calcnL_mag = pow((pow(tibia_r_pG[0] - calcn_l_pG[0], 2) + pow(tibia_r_pG[1] - calcn_l_pG[1], 2) + pow(tibia_r_pG[2] - calcn_l_pG[2], 2)), 0.5);
	osim_double_adouble tibiaL_calcnR_mag = pow((pow(tibia_l_pG[0] - calcn_r_pG[0], 2) + pow(tibia_l_pG[1] - calcn_r_pG[1], 2) + pow(tibia_l_pG[2] - calcn_r_pG[2], 2)), 0.5);

	// Tibia-Toes
	osim_double_adouble tibiaR_toesL_mag = pow((pow(tibia_r_pG[0] - toes_l_pG[0], 2) + pow(tibia_r_pG[1] - toes_l_pG[1], 2) + pow(tibia_r_pG[2] - toes_l_pG[2], 2)), 0.5);
	osim_double_adouble tibiaL_toesR_mag = pow((pow(tibia_l_pG[0] - toes_r_pG[0], 2) + pow(tibia_l_pG[1] - toes_r_pG[1], 2) + pow(tibia_l_pG[2] - toes_r_pG[2], 2)), 0.5);

	// Pelvis-Hand
	osim_double_adouble handR_pelvis_mag = pow((pow(hand_r_pG[0] - pelvis_pG[0], 2) + pow(hand_r_pG[1] - pelvis_pG[1], 2) + pow(hand_r_pG[2] - pelvis_pG[2], 2)), 0.5);
	osim_double_adouble handL_pelvis_mag = pow((pow(hand_l_pG[0] - pelvis_pG[0], 2) + pow(hand_l_pG[1] - pelvis_pG[1], 2) + pow(hand_l_pG[2] - pelvis_pG[2], 2)), 0.5);

	// Femur-Hand
	osim_double_adouble handR_femurR_mag = pow((pow(hand_r_pG[0] - femur_r_pG[0], 2) + pow(hand_r_pG[1] - femur_r_pG[1], 2) + pow(hand_r_pG[2] - femur_r_pG[2], 2)), 0.5);
	osim_double_adouble handL_femurL_mag = pow((pow(hand_l_pG[0] - femur_l_pG[0], 2) + pow(hand_l_pG[1] - femur_l_pG[1], 2) + pow(hand_l_pG[2] - femur_l_pG[2], 2)), 0.5);

	int ndof = NX / 2;

	// appliedMobilityForces
	Vector appliedMobilityForces(ndof);
	appliedMobilityForces.setToZero();

	// appliedBodyForces
	Vector_<SpatialVec> appliedBodyForces;
	int nbodies = model->getBodySet().getSize() + 1; // including ground
	appliedBodyForces.resize(nbodies);
	appliedBodyForces.setToZero();
	/// Gravity
	Vec3 gravity(0);
	gravity[1] = -9.81;

	// weights
	Vector_<SpatialVec> weights;
	weights.resize(nbodies);
	weights.setToZero();
	/// Add to model
	for (int i = 0; i < model->getBodySet().getSize(); ++i) {
		model->getMatterSubsystem().addInStationForce(*state, model->getBodySet().get(i).getMobilizedBodyIndex(), model->getBodySet().get(i).getMassCenter(), gravity*model->getBodySet().get(i).getMass(), weights);
	}

	Array<osim_double_adouble> r1Forces = rsph1->getRecordValues(*state);
	Array<osim_double_adouble> r2Forces = rsph2->getRecordValues(*state);
	Array<osim_double_adouble> r3Forces = rsph3->getRecordValues(*state);
	Array<osim_double_adouble> r4Forces = rsph4->getRecordValues(*state);
	Array<osim_double_adouble> r5Forces = rsph5->getRecordValues(*state);
	Array<osim_double_adouble> r6Forces = rsph6->getRecordValues(*state);
	Array<osim_double_adouble> r7Forces = rsph7->getRecordValues(*state);

	Array<osim_double_adouble> l1Forces = lsph1->getRecordValues(*state);
	Array<osim_double_adouble> l2Forces = lsph2->getRecordValues(*state);
	Array<osim_double_adouble> l3Forces = lsph3->getRecordValues(*state);
	Array<osim_double_adouble> l4Forces = lsph4->getRecordValues(*state);
	Array<osim_double_adouble> l5Forces = lsph5->getRecordValues(*state);
	Array<osim_double_adouble> l6Forces = lsph6->getRecordValues(*state);
	Array<osim_double_adouble> l7Forces = lsph7->getRecordValues(*state);
	// Each array is 12 by 1; first 6 terms: forces & moments @ ground; second 6 terms: forces and moments @ sphere


	SpatialVec r1Vec(0);
	r1Vec[0][0] = r1Forces[9];
	r1Vec[0][1] = r1Forces[10];
	r1Vec[0][2] = r1Forces[11];
	r1Vec[1][0] = r1Forces[6];
	r1Vec[1][1] = r1Forces[7];
	r1Vec[1][2] = r1Forces[8];

	SpatialVec r2Vec(0);
	r2Vec[0][0] = r2Forces[9];
	r2Vec[0][1] = r2Forces[10];
	r2Vec[0][2] = r2Forces[11];
	r2Vec[1][0] = r2Forces[6];
	r2Vec[1][1] = r2Forces[7];
	r2Vec[1][2] = r2Forces[8];

	SpatialVec r3Vec(0);
	r3Vec[0][0] = r3Forces[9];
	r3Vec[0][1] = r3Forces[10];
	r3Vec[0][2] = r3Forces[11];
	r3Vec[1][0] = r3Forces[6];
	r3Vec[1][1] = r3Forces[7];
	r3Vec[1][2] = r3Forces[8];

	SpatialVec r4Vec(0);
	r4Vec[0][0] = r4Forces[9];
	r4Vec[0][1] = r4Forces[10];
	r4Vec[0][2] = r4Forces[11];
	r4Vec[1][0] = r4Forces[6];
	r4Vec[1][1] = r4Forces[7];
	r4Vec[1][2] = r4Forces[8];

	SpatialVec r5Vec(0);
	r5Vec[0][0] = r5Forces[9];
	r5Vec[0][1] = r5Forces[10];
	r5Vec[0][2] = r5Forces[11];
	r5Vec[1][0] = r5Forces[6];
	r5Vec[1][1] = r5Forces[7];
	r5Vec[1][2] = r5Forces[8];

	SpatialVec r6Vec(0);
	r6Vec[0][0] = r6Forces[9];
	r6Vec[0][1] = r6Forces[10];
	r6Vec[0][2] = r6Forces[11];
	r6Vec[1][0] = r6Forces[6];
	r6Vec[1][1] = r6Forces[7];
	r6Vec[1][2] = r6Forces[8];

	SpatialVec r7Vec(0);
	r7Vec[0][0] = r7Forces[9];
	r7Vec[0][1] = r7Forces[10];
	r7Vec[0][2] = r7Forces[11];
	r7Vec[1][0] = r7Forces[6];
	r7Vec[1][1] = r7Forces[7];
	r7Vec[1][2] = r7Forces[8];


	SpatialVec l1Vec(0);
	l1Vec[0][0] = l1Forces[9];
	l1Vec[0][1] = l1Forces[10];
	l1Vec[0][2] = l1Forces[11];
	l1Vec[1][0] = l1Forces[6];
	l1Vec[1][1] = l1Forces[7];
	l1Vec[1][2] = l1Forces[8];

	SpatialVec l2Vec(0);
	l2Vec[0][0] = l2Forces[9];
	l2Vec[0][1] = l2Forces[10];
	l2Vec[0][2] = l2Forces[11];
	l2Vec[1][0] = l2Forces[6];
	l2Vec[1][1] = l2Forces[7];
	l2Vec[1][2] = l2Forces[8];

	SpatialVec l3Vec(0);
	l3Vec[0][0] = l3Forces[9];
	l3Vec[0][1] = l3Forces[10];
	l3Vec[0][2] = l3Forces[11];
	l3Vec[1][0] = l3Forces[6];
	l3Vec[1][1] = l3Forces[7];
	l3Vec[1][2] = l3Forces[8];

	SpatialVec l4Vec(0);
	l4Vec[0][0] = l4Forces[9];
	l4Vec[0][1] = l4Forces[10];
	l4Vec[0][2] = l4Forces[11];
	l4Vec[1][0] = l4Forces[6];
	l4Vec[1][1] = l4Forces[7];
	l4Vec[1][2] = l4Forces[8];

	SpatialVec l5Vec(0);
	l5Vec[0][0] = l5Forces[9];
	l5Vec[0][1] = l5Forces[10];
	l5Vec[0][2] = l5Forces[11];
	l5Vec[1][0] = l5Forces[6];
	l5Vec[1][1] = l5Forces[7];
	l5Vec[1][2] = l5Forces[8];

	SpatialVec l6Vec(0);
	l6Vec[0][0] = l6Forces[9];
	l6Vec[0][1] = l6Forces[10];
	l6Vec[0][2] = l6Forces[11];
	l6Vec[1][0] = l6Forces[6];
	l6Vec[1][1] = l6Forces[7];
	l6Vec[1][2] = l6Forces[8];

	SpatialVec l7Vec(0);
	l7Vec[0][0] = l7Forces[9];
	l7Vec[0][1] = l7Forces[10];
	l7Vec[0][2] = l7Forces[11];
	l7Vec[1][0] = l7Forces[6];
	l7Vec[1][1] = l7Forces[7];
	l7Vec[1][2] = l7Forces[8];

	// Alternatively
	//r6Vec[0] = Vec3(r6Forces[9],r6Forces[10],r6Forces[11]);
	//r6Vec[1] = Vec3(r6Forces[6],r6Forces[7],r6Forces[8]);

	int mbi_toesR = model->getBodySet().get("toes_r").getMobilizedBodyIndex();
	int mbi_calcnR = model->getBodySet().get("calcn_r").getMobilizedBodyIndex();

	int mbi_toesL = model->getBodySet().get("toes_l").getMobilizedBodyIndex();
	int mbi_calcnL = model->getBodySet().get("calcn_l").getMobilizedBodyIndex();


	//appliedBodyForces = weights;
	//appliedBodyForces[mbi_toesR] = appliedBodyForces[mbi_toesR] + r1Vec + r2Vec;
	//appliedBodyForces[mbi_calcnR] = appliedBodyForces[mbi_calcnR] + r3Vec + r4Vec + r5Vec + r6Vec;

	// Determine sphere contact position for application of GRF controls
	// First in global and then local frames
	Vec3 rSph1Pos_G = toes_r->findStationLocationInGround(*state, rSphPos1);
	Vec3 rSph1Pos_G_cont = rSph1Pos_G - radius * normal;
	Vec3 rSph1Pos_G_cont_adj = rSph1Pos_G_cont - 0.5*rSph1Pos_G_cont[1] * normal;
	Vec3 rSph1Pos_L_cont_adj = model->getGround().findStationLocationInAnotherFrame(*state, rSph1Pos_G_cont_adj, model->getBodySet().get("toes_r"));

	Vec3 rSph2Pos_G = toes_r->findStationLocationInGround(*state, rSphPos2);
	Vec3 rSph2Pos_G_cont = rSph2Pos_G - radius * normal;
	Vec3 rSph2Pos_G_cont_adj = rSph2Pos_G_cont - 0.5*rSph2Pos_G_cont[1] * normal;
	Vec3 rSph2Pos_L_cont_adj = model->getGround().findStationLocationInAnotherFrame(*state, rSph2Pos_G_cont_adj, model->getBodySet().get("toes_r"));

	Vec3 rSph3Pos_G = calcn_r->findStationLocationInGround(*state, rSphPos3);
	Vec3 rSph3Pos_G_cont = rSph3Pos_G - radius * normal;
	Vec3 rSph3Pos_G_cont_adj = rSph3Pos_G_cont - 0.5*rSph3Pos_G_cont[1] * normal;
	Vec3 rSph3Pos_L_cont_adj = model->getGround().findStationLocationInAnotherFrame(*state, rSph3Pos_G_cont_adj, model->getBodySet().get("calcn_r"));

	Vec3 rSph4Pos_G = calcn_r->findStationLocationInGround(*state, rSphPos4);
	Vec3 rSph4Pos_G_cont = rSph4Pos_G - radius * normal;
	Vec3 rSph4Pos_G_cont_adj = rSph4Pos_G_cont - 0.5*rSph4Pos_G_cont[1] * normal;
	Vec3 rSph4Pos_L_cont_adj = model->getGround().findStationLocationInAnotherFrame(*state, rSph4Pos_G_cont_adj, model->getBodySet().get("calcn_r"));

	Vec3 rSph5Pos_G = calcn_r->findStationLocationInGround(*state, rSphPos5);
	Vec3 rSph5Pos_G_cont = rSph5Pos_G - radius * normal;
	Vec3 rSph5Pos_G_cont_adj = rSph5Pos_G_cont - 0.5*rSph5Pos_G_cont[1] * normal;
	Vec3 rSph5Pos_L_cont_adj = model->getGround().findStationLocationInAnotherFrame(*state, rSph5Pos_G_cont_adj, model->getBodySet().get("calcn_r"));

	Vec3 rSph6Pos_G = calcn_r->findStationLocationInGround(*state, rSphPos6);
	Vec3 rSph6Pos_G_cont = rSph6Pos_G - radius * normal;
	Vec3 rSph6Pos_G_cont_adj = rSph6Pos_G_cont - 0.5*rSph6Pos_G_cont[1] * normal;
	Vec3 rSph6Pos_L_cont_adj = model->getGround().findStationLocationInAnotherFrame(*state, rSph6Pos_G_cont_adj, model->getBodySet().get("calcn_r"));

	Vec3 rSph7Pos_G = calcn_r->findStationLocationInGround(*state, rSphPos7);
	Vec3 rSph7Pos_G_cont = rSph7Pos_G - radius * normal;
	Vec3 rSph7Pos_G_cont_adj = rSph7Pos_G_cont - 0.5*rSph7Pos_G_cont[1] * normal;
	Vec3 rSph7Pos_L_cont_adj = model->getGround().findStationLocationInAnotherFrame(*state, rSph7Pos_G_cont_adj, model->getBodySet().get("calcn_r"));


	Vec3 lSph1Pos_G = toes_l->findStationLocationInGround(*state, lSphPos1);
	Vec3 lSph1Pos_G_cont = lSph1Pos_G - radius * normal;
	Vec3 lSph1Pos_G_cont_adj = lSph1Pos_G_cont - 0.5*lSph1Pos_G_cont[1] * normal;
	Vec3 lSph1Pos_L_cont_adj = model->getGround().findStationLocationInAnotherFrame(*state, lSph1Pos_G_cont_adj, model->getBodySet().get("toes_l"));

	Vec3 lSph2Pos_G = toes_l->findStationLocationInGround(*state, lSphPos2);
	Vec3 lSph2Pos_G_cont = lSph2Pos_G - radius * normal;
	Vec3 lSph2Pos_G_cont_adj = lSph2Pos_G_cont - 0.5*lSph2Pos_G_cont[1] * normal;
	Vec3 lSph2Pos_L_cont_adj = model->getGround().findStationLocationInAnotherFrame(*state, lSph2Pos_G_cont_adj, model->getBodySet().get("toes_l"));

	Vec3 lSph3Pos_G = calcn_l->findStationLocationInGround(*state, lSphPos3);
	Vec3 lSph3Pos_G_cont = lSph3Pos_G - radius * normal;
	Vec3 lSph3Pos_G_cont_adj = lSph3Pos_G_cont - 0.5*lSph3Pos_G_cont[1] * normal;
	Vec3 lSph3Pos_L_cont_adj = model->getGround().findStationLocationInAnotherFrame(*state, lSph3Pos_G_cont_adj, model->getBodySet().get("calcn_l"));

	Vec3 lSph4Pos_G = calcn_l->findStationLocationInGround(*state, lSphPos4);
	Vec3 lSph4Pos_G_cont = lSph4Pos_G - radius * normal;
	Vec3 lSph4Pos_G_cont_adj = lSph4Pos_G_cont - 0.5*lSph4Pos_G_cont[1] * normal;
	Vec3 lSph4Pos_L_cont_adj = model->getGround().findStationLocationInAnotherFrame(*state, lSph4Pos_G_cont_adj, model->getBodySet().get("calcn_l"));

	Vec3 lSph5Pos_G = calcn_l->findStationLocationInGround(*state, lSphPos5);
	Vec3 lSph5Pos_G_cont = lSph5Pos_G - radius * normal;
	Vec3 lSph5Pos_G_cont_adj = lSph5Pos_G_cont - 0.5*lSph5Pos_G_cont[1] * normal;
	Vec3 lSph5Pos_L_cont_adj = model->getGround().findStationLocationInAnotherFrame(*state, lSph5Pos_G_cont_adj, model->getBodySet().get("calcn_l"));

	Vec3 lSph6Pos_G = calcn_l->findStationLocationInGround(*state, lSphPos6);
	Vec3 lSph6Pos_G_cont = lSph6Pos_G - radius * normal;
	Vec3 lSph6Pos_G_cont_adj = lSph6Pos_G_cont - 0.5*lSph6Pos_G_cont[1] * normal;
	Vec3 lSph6Pos_L_cont_adj = model->getGround().findStationLocationInAnotherFrame(*state, lSph6Pos_G_cont_adj, model->getBodySet().get("calcn_l"));

	Vec3 lSph7Pos_G = calcn_l->findStationLocationInGround(*state, lSphPos7);
	Vec3 lSph7Pos_G_cont = lSph7Pos_G - radius * normal;
	Vec3 lSph7Pos_G_cont_adj = lSph7Pos_G_cont - 0.5*lSph7Pos_G_cont[1] * normal;
	Vec3 lSph7Pos_L_cont_adj = model->getGround().findStationLocationInAnotherFrame(*state, lSph7Pos_G_cont_adj, model->getBodySet().get("calcn_l"));


	Vector_<SpatialVec> groundForces(nbodies);
	groundForces.setToZero();

	// Update Vector of Spatial Vectors with GRF controls
	model->getMatterSubsystem().addInStationForce(*state, model->getBodySet().get("toes_r").getMobilizedBodyIndex(), rSph1Pos_L_cont_adj, Vec3(r1Vec[1][0],r1Vec[1][1],r1Vec[1][2]), groundForces);
	model->getMatterSubsystem().addInStationForce(*state, model->getBodySet().get("toes_r").getMobilizedBodyIndex(), rSph2Pos_L_cont_adj, Vec3(r2Vec[1][0], r2Vec[1][1], r2Vec[1][2]), groundForces);
	model->getMatterSubsystem().addInStationForce(*state, model->getBodySet().get("calcn_r").getMobilizedBodyIndex(), rSph3Pos_L_cont_adj, Vec3(r3Vec[1][0], r3Vec[1][1], r3Vec[1][2]), groundForces);
	model->getMatterSubsystem().addInStationForce(*state, model->getBodySet().get("calcn_r").getMobilizedBodyIndex(), rSph4Pos_L_cont_adj, Vec3(r4Vec[1][0], r4Vec[1][1], r4Vec[1][2]), groundForces);
	model->getMatterSubsystem().addInStationForce(*state, model->getBodySet().get("calcn_r").getMobilizedBodyIndex(), rSph5Pos_L_cont_adj, Vec3(r5Vec[1][0], r5Vec[1][1], r5Vec[1][2]), groundForces);
	model->getMatterSubsystem().addInStationForce(*state, model->getBodySet().get("calcn_r").getMobilizedBodyIndex(), rSph6Pos_L_cont_adj, Vec3(r6Vec[1][0], r6Vec[1][1], r6Vec[1][2]), groundForces);
	model->getMatterSubsystem().addInStationForce(*state, model->getBodySet().get("calcn_r").getMobilizedBodyIndex(), rSph7Pos_L_cont_adj, Vec3(r7Vec[1][0], r7Vec[1][1], r7Vec[1][2]), groundForces);

	model->getMatterSubsystem().addInStationForce(*state, model->getBodySet().get("toes_l").getMobilizedBodyIndex(), lSph1Pos_L_cont_adj, Vec3(l1Vec[1][0], l1Vec[1][1], l1Vec[1][2]), groundForces);
	model->getMatterSubsystem().addInStationForce(*state, model->getBodySet().get("toes_l").getMobilizedBodyIndex(), lSph2Pos_L_cont_adj, Vec3(l2Vec[1][0], l2Vec[1][1], l2Vec[1][2]), groundForces);
	model->getMatterSubsystem().addInStationForce(*state, model->getBodySet().get("calcn_l").getMobilizedBodyIndex(), lSph3Pos_L_cont_adj, Vec3(l3Vec[1][0], l3Vec[1][1], l3Vec[1][2]), groundForces);
	model->getMatterSubsystem().addInStationForce(*state, model->getBodySet().get("calcn_l").getMobilizedBodyIndex(), lSph4Pos_L_cont_adj, Vec3(l4Vec[1][0], l4Vec[1][1], l4Vec[1][2]), groundForces);
	model->getMatterSubsystem().addInStationForce(*state, model->getBodySet().get("calcn_l").getMobilizedBodyIndex(), lSph5Pos_L_cont_adj, Vec3(l5Vec[1][0], l5Vec[1][1], l5Vec[1][2]), groundForces);
	model->getMatterSubsystem().addInStationForce(*state, model->getBodySet().get("calcn_l").getMobilizedBodyIndex(), lSph6Pos_L_cont_adj, Vec3(l6Vec[1][0], l6Vec[1][1], l6Vec[1][2]), groundForces);
	model->getMatterSubsystem().addInStationForce(*state, model->getBodySet().get("calcn_l").getMobilizedBodyIndex(), lSph7Pos_L_cont_adj, Vec3(l7Vec[1][0], l7Vec[1][1], l7Vec[1][2]), groundForces);


	// Air Drag
	Vector_<SpatialVec> airDrag_matrix;
	airDrag_matrix.resize(nbodies);
	airDrag_matrix.setToZero();

	Vec3 airDrag;
	airDrag.setToZero();

	// Express system's COM in pelvis' local frame
	Vec3 COM_p_L = model->getGround().findStationLocationInAnotherFrame(*state, COM_p_G, model->getBodySet().get("pelvis"));

	// Air drag calculations from Samozino et al. 2016
	osim_double_adouble Cd = 0.9;
	osim_double_adouble Af = 0.50641133;
	osim_double_adouble rho = 1.20474061;

	osim_double_adouble airFric_k = 0.5*rho*Af*Cd;

	airDrag(0) = -airFric_k * (COM_v_G(0)*COM_v_G(0));

	model->getMatterSubsystem().addInStationForce(*state, model->getBodySet().get("pelvis").getMobilizedBodyIndex(), COM_p_L, airDrag, airDrag_matrix);


	//appliedBodyForces = weights + groundForces + airDrag_matrix;
	appliedBodyForces = weights + airDrag_matrix + groundForces;

	//appliedBodyForces[mbi_toesR] = appliedBodyForces[mbi_toesR] + r1Vec + r2Vec;
	//appliedBodyForces[mbi_calcnR] = appliedBodyForces[mbi_calcnR] + r3Vec + r4Vec + r5Vec + r6Vec;

	//appliedBodyForces[mbi_toesL] = appliedBodyForces[mbi_toesL] + l1Vec + l2Vec;
	//appliedBodyForces[mbi_calcnL] = appliedBodyForces[mbi_calcnL] + l3Vec + l4Vec + l5Vec + l6Vec;



	// knownUdot
	Vector knownUdot(ndof);
	knownUdot.setToZero();
	//for (int i = 0; i < NDOF; ++i) knownUdot[i] = ua[i];  
	for (int i = 0; i < 9; ++i) knownUdot[i] = ua[i];
	knownUdot[15] = ua[9];
	knownUdot[23] = ua[10];
	knownUdot[27] = ua[11];
	knownUdot[31] = ua[12];
	knownUdot[9] = ua[13];
	knownUdot[10] = ua[14];
	knownUdot[11] = ua[15];
	knownUdot[16] = ua[16];
	knownUdot[24] = ua[17];
	knownUdot[28] = ua[18];
	knownUdot[32] = ua[19];
	knownUdot[12] = ua[20];
	knownUdot[13] = ua[21];
	knownUdot[14] = ua[22];
	knownUdot[17] = ua[23];
	knownUdot[18] = ua[24];
	knownUdot[19] = ua[25];
	knownUdot[25] = ua[26];
	knownUdot[29] = ua[27];
	knownUdot[33] = ua[28];
	knownUdot[34] = ua[29];
	knownUdot[20] = ua[30];
	knownUdot[21] = ua[31];
	knownUdot[22] = ua[32];
	knownUdot[26] = ua[33];
	knownUdot[30] = ua[34];
	knownUdot[35] = ua[35];
	knownUdot[36] = ua[36];

	// residualMobilityForces
	Vector residualMobilityForces(NR);
	residualMobilityForces.setToZero();

	model->getMatterSubsystem().calcResidualForceIgnoringConstraints(*state, appliedMobilityForces, appliedBodyForces, knownUdot, residualMobilityForces);

	// Again, necessary due to mismatch 
	Vector ordResidMobForces(NR);
	ordResidMobForces.setToZero();

	for (int i = 0; i < 9; ++i) ordResidMobForces[i] = residualMobilityForces[i];
	ordResidMobForces[9] = residualMobilityForces[15];
	ordResidMobForces[10] = residualMobilityForces[23];
	ordResidMobForces[11] = residualMobilityForces[27];
	ordResidMobForces[12] = residualMobilityForces[31];
	ordResidMobForces[13] = residualMobilityForces[9];
	ordResidMobForces[14] = residualMobilityForces[10];
	ordResidMobForces[15] = residualMobilityForces[11];
	ordResidMobForces[16] = residualMobilityForces[16];
	ordResidMobForces[17] = residualMobilityForces[24];
	ordResidMobForces[18] = residualMobilityForces[28];
	ordResidMobForces[19] = residualMobilityForces[32];
	ordResidMobForces[20] = residualMobilityForces[12];
	ordResidMobForces[21] = residualMobilityForces[13];
	ordResidMobForces[22] = residualMobilityForces[14];
	ordResidMobForces[23] = residualMobilityForces[17];
	ordResidMobForces[24] = residualMobilityForces[18];
	ordResidMobForces[25] = residualMobilityForces[19];
	ordResidMobForces[26] = residualMobilityForces[25];
	ordResidMobForces[27] = residualMobilityForces[29];
	ordResidMobForces[28] = residualMobilityForces[33];
	ordResidMobForces[29] = residualMobilityForces[34];
	ordResidMobForces[30] = residualMobilityForces[20];
	ordResidMobForces[31] = residualMobilityForces[21];
	ordResidMobForces[32] = residualMobilityForces[22];
	ordResidMobForces[33] = residualMobilityForces[26];
	ordResidMobForces[34] = residualMobilityForces[30];
	ordResidMobForces[35] = residualMobilityForces[35];
	ordResidMobForces[36] = residualMobilityForces[36];

	// CasADi may not always request all outputs
	// if res[i] is a null pointer, this means that output i is not required
	for (int i = 0; i < NR; ++i) {
		res[0][i] = value<T>(ordResidMobForces[i]);
	}
	for (int i = 0; i < 3; ++i) res[0][i + NDOF] = value<T>(r1Vec[1][i]);
	for (int i = 0; i < 3; ++i) res[0][i + NDOF + 3] = value<T>(r2Vec[1][i]);
	for (int i = 0; i < 3; ++i) res[0][i + NDOF + 6] = value<T>(r3Vec[1][i]);
	for (int i = 0; i < 3; ++i) res[0][i + NDOF + 9] = value<T>(r4Vec[1][i]);
	for (int i = 0; i < 3; ++i) res[0][i + NDOF + 12] = value<T>(r5Vec[1][i]);
	for (int i = 0; i < 3; ++i) res[0][i + NDOF + 15] = value<T>(r6Vec[1][i]);
	for (int i = 0; i < 3; ++i) res[0][i + NDOF + 18] = value<T>(r7Vec[1][i]);
	for (int i = 0; i < 3; ++i) res[0][i + NDOF + 21] = value<T>(l1Vec[1][i]);
	for (int i = 0; i < 3; ++i) res[0][i + NDOF + 24] = value<T>(l2Vec[1][i]);
	for (int i = 0; i < 3; ++i) res[0][i + NDOF + 27] = value<T>(l3Vec[1][i]);
	for (int i = 0; i < 3; ++i) res[0][i + NDOF + 30] = value<T>(l4Vec[1][i]);
	for (int i = 0; i < 3; ++i) res[0][i + NDOF + 33] = value<T>(l5Vec[1][i]);
	for (int i = 0; i < 3; ++i) res[0][i + NDOF + 36] = value<T>(l6Vec[1][i]);
	for (int i = 0; i < 3; ++i) res[0][i + NDOF + 39] = value<T>(l7Vec[1][i]);

	for (int i = 0; i < 3; ++i) res[0][i + NDOF + 42] = value<T>(COM_p_G[i]);
	for (int i = 0; i < 3; ++i) res[0][i + NDOF + 45] = value<T>(COM_v_G[i]);

	for (int i = 0; i < 3; ++i) res[0][i + NDOF + 48] = value<T>(calcn_r_pG[i]); // R Calcaneus Frame Position in Ground
	for (int i = 0; i < 3; ++i) res[0][i + NDOF + 51] = value<T>(calcn_r_vG[i]); // R Calcaneus Frame Velocity in Ground
	for (int i = 0; i < 3; ++i) res[0][i + NDOF + 54] = value<T>(calcn_l_pG[i]); // L Calcaneus Frame Position in Ground
	for (int i = 0; i < 3; ++i) res[0][i + NDOF + 57] = value<T>(calcn_l_vG[i]); // L Calcaneus Frame Velocity in Ground
	for (int i = 0; i < 3; ++i) res[0][i + NDOF + 60] = value<T>(toes_r_pG[i]); // R Toes Frame Position in Ground
	for (int i = 0; i < 3; ++i) res[0][i + NDOF + 63] = value<T>(toes_r_vG[i]); // R Toes Frame Velocity in Ground
	for (int i = 0; i < 3; ++i) res[0][i + NDOF + 66] = value<T>(toes_l_pG[i]); // L Toes Frame Position in Ground
	for (int i = 0; i < 3; ++i) res[0][i + NDOF + 69] = value<T>(toes_l_vG[i]); // L Toes Frame Velocity in Ground

	res[0][NDOF + 72] = value<T>(calcn_mag); // distance; Calcaneus
	res[0][NDOF + 73] = value<T>(toes_mag); // distance; Toes
	res[0][NDOF + 74] = value<T>(tibia_mag); // distance; Tibia
	res[0][NDOF + 75] = value<T>(radius_mag); // distance; Radius
	res[0][NDOF + 76] = value<T>(hand_mag); // distance; Hand
	res[0][NDOF + 77] = value<T>(torsoRadius_r_mag); // distance; Torso-Radius R
	res[0][NDOF + 78] = value<T>(torsoRadius_l_mag); // distance; Torso-Radius L
	res[0][NDOF + 79] = value<T>(torsoHand_r_mag); // distance; Torso-Hand R
	res[0][NDOF + 80] = value<T>(torsoHand_l_mag); // distance; Torso-Hand L
	res[0][NDOF + 81] = value<T>(tibiaL_calcnR_mag); // distance; Tibia L - Calcaneus R
	res[0][NDOF + 82] = value<T>(tibiaR_calcnL_mag); // distance; Tibia R - Calcaneus L
	res[0][NDOF + 83] = value<T>(tibiaL_toesR_mag); // distance; Tibia L - Toes R
	res[0][NDOF + 84] = value<T>(tibiaR_toesL_mag); // distance; Tibia R - Toes L
	res[0][NDOF + 85] = value<T>(handR_pelvis_mag); // distance; Hand R - Pelvis
	res[0][NDOF + 86] = value<T>(handL_pelvis_mag); // distance; Hand L - Pelvis
	res[0][NDOF + 87] = value<T>(handR_femurR_mag); // distance; Hand R - Femur R
	res[0][NDOF + 88] = value<T>(handL_femurL_mag); // distance; Hand L - Femur L
	res[0][NDOF + 89] = value<T>(pelvisRadius_r_mag); // distance; Pelvis-Radius R
	res[0][NDOF + 90] = value<T>(pelvisRadius_l_mag); // distance; Pelvis-Radius L

	for (int i = 0; i < 3; ++i) res[0][i + NDOF + 91] = value<T>(femur_r_XYZ[i]); 
	for (int i = 0; i < 3; ++i) res[0][i + NDOF + 94] = value<T>(femur_l_XYZ[i]);
	for (int i = 0; i < 3; ++i) res[0][i + NDOF + 97] = value<T>(tibia_r_XYZ[i]);
	for (int i = 0; i < 3; ++i) res[0][i + NDOF + 100] = value<T>(tibia_l_XYZ[i]);
	for (int i = 0; i < 3; ++i) res[0][i + NDOF + 103] = value<T>(torso_XYZ[i]);

	for (int i = 0; i < 3; ++i) res[0][i + NDOF + 106] = value<T>(femur_r_H_COM[i]);
	for (int i = 0; i < 3; ++i) res[0][i + NDOF + 109] = value<T>(femur_l_H_COM[i]);
	for (int i = 0; i < 3; ++i) res[0][i + NDOF + 112] = value<T>(tibia_r_H_COM[i]);
	for (int i = 0; i < 3; ++i) res[0][i + NDOF + 115] = value<T>(tibia_l_H_COM[i]);
	for (int i = 0; i < 3; ++i) res[0][i + NDOF + 118] = value<T>(talus_r_H_COM[i]);
	for (int i = 0; i < 3; ++i) res[0][i + NDOF + 121] = value<T>(talus_l_H_COM[i]);
	for (int i = 0; i < 3; ++i) res[0][i + NDOF + 124] = value<T>(calcn_r_H_COM[i]);
	for (int i = 0; i < 3; ++i) res[0][i + NDOF + 127] = value<T>(calcn_l_H_COM[i]);
	for (int i = 0; i < 3; ++i) res[0][i + NDOF + 130] = value<T>(toes_r_H_COM[i]);
	for (int i = 0; i < 3; ++i) res[0][i + NDOF + 133] = value<T>(toes_l_H_COM[i]);
	for (int i = 0; i < 3; ++i) res[0][i + NDOF + 136] = value<T>(humerus_r_H_COM[i]);
	for (int i = 0; i < 3; ++i) res[0][i + NDOF + 139] = value<T>(humerus_l_H_COM[i]);
	for (int i = 0; i < 3; ++i) res[0][i + NDOF + 142] = value<T>(radius_r_H_COM[i]);
	for (int i = 0; i < 3; ++i) res[0][i + NDOF + 145] = value<T>(radius_l_H_COM[i]);
	for (int i = 0; i < 3; ++i) res[0][i + NDOF + 148] = value<T>(ulna_r_H_COM[i]);
	for (int i = 0; i < 3; ++i) res[0][i + NDOF + 151] = value<T>(ulna_l_H_COM[i]);
	for (int i = 0; i < 3; ++i) res[0][i + NDOF + 154] = value<T>(hand_r_H_COM[i]);
	for (int i = 0; i < 3; ++i) res[0][i + NDOF + 157] = value<T>(hand_l_H_COM[i]);
	for (int i = 0; i < 3; ++i) res[0][i + NDOF + 160] = value<T>(pelvis_H_COM[i]);
	for (int i = 0; i < 3; ++i) res[0][i + NDOF + 163] = value<T>(torso_H_COM[i]);


	return 0;

}

int main() {

	Recorder x[NX];
	Recorder u[NU];
	Recorder p[NP];
	Recorder tau[NR + 42 + 124];

	for (int i = 0; i < NX; ++i) x[i] <<= 0;
	for (int i = 0; i < NU; ++i) u[i] <<= 0;
	for (int i = 0; i < NP; ++i) p[i] <<= 0;


	const Recorder* Recorder_arg[n_in] = { x,u, p };
	Recorder* Recorder_res[n_out] = { tau };

	F_generic<Recorder>(Recorder_arg, Recorder_res);

	//NR+36+49; number of torques and GRF from contact model and positions of landmarks 
	double res[NR + 42 + 124];
	for (int i = 0; i < NR + 42 + 124; ++i) Recorder_res[0][i] >>= res[i];

	Recorder::stop_recording();

	return 0;
}
