#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"
#include <iostream>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class Tools {
private:
	ofstream  L_out, R_out;
public:

	

  /**
  * Constructor.
  */
  Tools();

  /**
  * Destructor.
  */
  virtual ~Tools();

  /**
  * A helper method to calculate RMSE.
  */
  VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);
  void R_write_line(double NIS_R);
  void L_write_line(double NIS_L);

};

#endif /* TOOLS_H_ */
