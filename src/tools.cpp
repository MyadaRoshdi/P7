#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {

	



}

Tools::~Tools() {
	
	L_out.open("L_out.txt", ios::out | ios::ate | ios::app);
	R_out.open("R_out.txt", ios::out | ios::ate | ios::app);
}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
	    * Calculate the RMSE here.
	 */

	VectorXd rmse(4);

	rmse << 0,0,0,0;

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	if(estimations.size() != ground_truth.size() || estimations.size() == 0){

		cout << "Invalid estimation or ground_truth data" << endl;
		return rmse;
	}

	//accumulate squared residuals
	for(unsigned int i=0; i < estimations.size(); ++i){

		VectorXd residual = estimations[i] - ground_truth[i];

		//coefficient-wise multiplication
		residual = residual.array()*residual.array();
		rmse += residual;
	}

	//calculate the mean
	rmse = rmse/estimations.size();

	//calculate the squared root
	rmse = rmse.array().sqrt();

	//return the result
	return rmse;

}

void Tools::L_write_line(double NIS_L)
{
	if (L_out.is_open()){

		L_out << NIS_L << ",";
	}
	else{

		cout << "Cannot open L_out file!!!" << endl;
	}
	
}

void Tools::R_write_line(double NIS_R)
{
	if (R_out.is_open()) {

		R_out << NIS_R << ",";
	}
	else{

		cout << "Cannot open R_out file!!!" << endl;
	}
}

