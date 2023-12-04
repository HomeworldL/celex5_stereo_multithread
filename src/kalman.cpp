#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <cmath>
#include <limits>//������������ֲ�����
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "catch_prediction.h"

using namespace std;
using namespace Eigen;


vector<vector<double>> read_txt(const std::string fileName)
{
	ifstream myfile(fileName);

	if(!myfile.is_open () )
    {
        std::cout << "Open file failure" << std::endl;
    }        
    else
    {
        std::cout << "Open file succesed" << std::endl;
    }
	
	string temp;
	vector<vector<double>> result;
	vector<double> res;
	while (getline(myfile, temp))
	{
		
		stringstream input(temp);
		string out;
		while (input >> out)
		{
			res.push_back(stod(out));			
		}
		// cout<<endl;
		result.push_back(res);
		res.clear();
	}

	return result;
}



MatrixXd kalmanFilter(vector<vector<double>>& data)
{
	//���⼸������
	double Q = 0.00009; //Q�󣬲����࣬QС��ƫֱ��
	double R = 1;
	double N = 1;


	double X = 0, P = 1, A = 1, H = 1;

	// cout<<data.size()<<endl;
	

	MatrixXd output;
	output.setZero(data.size(), data[0].size());

	// cout<<1<<endl;

	double X_k, P_k, Kg, z_k;
	srand((int)time(0));

	
	for (int i = N; i < data.size(); i++)
	{
		X_k = A * X;
		P_k = A * P * 1.0 / A + Q;
		Kg = P_k * 1.0 / H / (H * P_k * 1.0 / H + R);

		//cout << rand() << endl;
		//cout << rand() * N << endl;
		//cout << round(rand() * N) << endl;

		int tmp = i - round(rand() / RAND_MAX * N);
		//cout << tmp << endl;
		//cout << data[tmp][0] << endl;

		z_k = data[tmp][0];
		X = X_k + Kg * (z_k - H * X_k);
		P = (1 - Kg * H) * P_k;
		output(i, 0) = X;
	}

	return output;

}



int main(int argc, char **argv) 
{
  	ros::init(argc, argv, "kalman_test");

	ofstream fout("/home/guoyeye/celex_ws/result.txt");

	// cout<<1<<endl;
	
	vector<vector<double>> data;
	MatrixXd output;

	data = read_txt("/home/guoyeye/celex_ws/disp2.txt");

	
	output = kalmanFilter(data);

	//cout << output << endl;
	// cout<<1<<endl;

	for (int i = 0; i < output.rows(); i++) 
	{
		for (int j = 0; j < output.cols(); j++)
		{
			fout << output(i, j) << endl; //����˲�������ݵ�txt�ĵ�������matlab��ͼ
		}
	}
	fout.close();


	//system("pause");
	return 0;


}