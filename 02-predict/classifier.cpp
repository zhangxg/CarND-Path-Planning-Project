#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include "classifier.h"


using namespace std;

/**
 * Initializes GNB
 */
GNB::GNB() {

}

GNB::~GNB() {}

void GNB::train(vector<vector<double>> data, vector<string> labels)
{

	/*
		Trains the classifier with N data points and labels.

		INPUTS
		data - array of N observations
		  - Each observation is a tuple with 4 values: s, d, 
		    s_dot and d_dot.
		  - Example : [
			  	[3.5, 0.1, 5.9, -0.02],
			  	[8.0, -0.3, 3.0, 2.2],
			  	...
		  	]

		labels - array of N labels
		  - Each label is one of "left", "keep", or "right".
	*/

	// the difficults: 
	// the data structures, is there a strucutre like python dictionary?
	// 

	// segment the training data
	std::vector<std::vector<double>> left_data;
	std::vector<std::vector<double>> keep_lane_data;
	std::vector<std::vector<double>> right_data;

	for (int i = 0; i < labels.size(); ++i)
	{
		if (labels[i] == "left")
		{
			left_data.push_back(data[i]);
		} 
		else if (labels[i] == "keep") {
			keep_lane_data.push_back(data[i]);
		}
		else if (labels[i] == "right") {
			right_data.push_back(data[i]);
		}
	}

	// calculate the mean and standard deviation
	std::vector<std::vector<double>> left_params;
	std::vector<std::vector<double>> keep_lane_params;
	std::vector<std::vector<double>> right_params;

	int size = left_data.size();
	double mean[4] = {0, 0, 0, 0};
	for (int i = 0; i < size; ++i)
	{
		for (int i = 0; i < left_data[i].size(); ++i)
		{
			mean[0] += left_data[i][0];
			mean[1] += left_data[i][1];
			mean[2] += left_data[i][2];
			mean[3] += left_data[i][3];
		}
	}

	std::vector<double> v;
	for (auto i : mean)
	{
		v.push_back(i / size);
	}
	left_params.push_back(v);

	// the standard deviation
	double square_sum[4] = {0, 0, 0, 0};
	for (int i = 0; i < left_data.size(); ++i)
	{
		for (int i = 0; i < left_data[i].size(); ++i)
		{
			square_sum[0] += (left_data[i][0] - mean[0]) * (left_data[i][0] - mean[0]);
			square_sum[1] += (left_data[i][1] - mean[1]) * (left_data[i][1] - mean[1]);
			square_sum[2] += (left_data[i][2] - mean[2]) * (left_data[i][2] - mean[2]);
			square_sum[3] += (left_data[i][3] - mean[3]) * (left_data[i][3] - mean[3]);
		}
	}

	std::vector<double> v2;
	for (auto i : square_sum) {
		v2.push_back(i / (size - 1));
	}
	left_params.push_back(v2);

	for (int i = 0; i < left_params.size(); ++i)
	{
		cout << left_params[i][0] << ", " << left_params[i][1] << ", " << left_params[i][2] << ", " << left_params[i][3] << endl;
	}

}

string GNB::predict(vector<double> sample)
{
	/*
		Once trained, this method is called and expected to return 
		a predicted behavior for the given observation.

		INPUTS

		observation - a 4 tuple with s, d, s_dot, d_dot.
		  - Example: [3.5, 0.1, 8.5, -0.2]

		OUTPUT

		A label representing the best guess of the classifier. Can
		be one of "left", "keep" or "right".
		"""
		# TODO - complete this
	*/

	return this->possible_labels[1];

}