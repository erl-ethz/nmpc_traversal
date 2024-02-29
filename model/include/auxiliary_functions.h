#include <nlohmann/json.hpp>
#include <fstream>

void load_spline(const int &num_splines, SplineLib::cSpline3 splines[]){
	std::ifstream strSpline("configuration/splines.json");
	nlohmann::json jsonSpline;
	jsonSpline << strSpline;
	for (int i = 0; i < num_splines; i++)
	{
		splines[i].xb.w = static_cast<float>(jsonSpline["Xb"]["w"][i]);
		splines[i].yb.w = static_cast<float>(jsonSpline["Yb"]["w"][i]);
		splines[i].zb.w = static_cast<float>(jsonSpline["Zb"]["w"][i]);
		splines[i].xb.x = static_cast<float>(jsonSpline["Xb"]["x"][i]);
		splines[i].yb.x = static_cast<float>(jsonSpline["Yb"]["x"][i]);
		splines[i].zb.x = static_cast<float>(jsonSpline["Zb"]["x"][i]);
		splines[i].xb.y = static_cast<float>(jsonSpline["Xb"]["y"][i]);
		splines[i].yb.y = static_cast<float>(jsonSpline["Yb"]["y"][i]);
		splines[i].zb.y = static_cast<float>(jsonSpline["Zb"]["y"][i]);
		splines[i].xb.z = static_cast<float>(jsonSpline["Xb"]["z"][i]);
		splines[i].yb.z = static_cast<float>(jsonSpline["Yb"]["z"][i]);
		splines[i].zb.z = static_cast<float>(jsonSpline["Zb"]["z"][i]);
	}
}

void FindReference_DotProduct(double* input, double* output, void* user_data)
{
	SplineLib::Vec3f current_pos_spline_lib{
		static_cast<float>(input[0]),
		static_cast<float>(input[1]), 
		static_cast<float>(input[2])};
	int number_splines = static_cast<int>(input[6]);
	SplineLib::cSpline3 splines[number_splines];

	for (int i = 0; i < number_splines; i++)
	{
		splines[i].xb.w = static_cast<float>(input[7+i*12]);
		splines[i].yb.w = static_cast<float>(input[8+i*12]);
		splines[i].zb.w = static_cast<float>(input[9+i*12]);
		splines[i].xb.x = static_cast<float>(input[10+i*12]);
		splines[i].yb.x = static_cast<float>(input[11+i*12]);
		splines[i].zb.x = static_cast<float>(input[12+i*12]);
		splines[i].xb.y = static_cast<float>(input[13+i*12]);
		splines[i].yb.y = static_cast<float>(input[14+i*12]);
		splines[i].zb.y = static_cast<float>(input[15+i*12]);
		splines[i].xb.z = static_cast<float>(input[16+i*12]);
		splines[i].yb.z = static_cast<float>(input[17+i*12]);
		splines[i].zb.z = static_cast<float>(input[18+i*12]);
	}
	
	int index;
	float t = SplineLib::FindClosestPoint(current_pos_spline_lib, number_splines, splines, &index);
	SplineLib::Vec3f pos_spline = SplineLib::Position(splines[index], t);
	SplineLib::Vec3f vel_spline = SplineLib::Velocity(splines[index], t);
	output[0] = static_cast<double>(pos_spline.x);
	output[1] = static_cast<double>(pos_spline.y);
	output[2] = static_cast<double>(pos_spline.z);

	// dot product
	double v_spline_x = static_cast<double>(vel_spline.x);
	double v_spline_y = static_cast<double>(vel_spline.y);
	double v_spline_z = static_cast<double>(vel_spline.z);
	double v_spline_norm_tmp = v_spline_x*v_spline_x + v_spline_y*v_spline_y + v_spline_z*v_spline_z;
	double v_spline_norm = (v_spline_norm_tmp>0.01) ? v_spline_norm_tmp : 0.01;
	output[3] = v_spline_x/v_spline_norm;
	output[4] = v_spline_y/v_spline_norm;
	output[5] = v_spline_z/v_spline_norm;
}
