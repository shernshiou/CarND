#include <iostream>
#include <sstream>
#include <fstream>
#include <algorithm>
#include <math.h>
#include <vector>
#include "classifier.h"

/**
 * Initializes GNB
 */
GNB::GNB()
  :model(possible_labels.size())
{

}

GNB::~GNB() {}

void GNB::train(vector<vector<double>> data, vector<string> labels)
{
  // Group classes
  vector<vector<vector<double>>> classes(possible_labels.size());
  for (int i=0; i<data.size(); i++) {
    auto pos = find(possible_labels.begin(), possible_labels.end(), labels[i]) - possible_labels.begin();
    classes[pos].push_back(data[i]);
  }

  // Build model
  for (int i=0; i<possible_labels.size(); i++) { // left, keep, right
    for (int j=0; j<data[0].size(); j++) { // s, d, s_dot, d_dot
      double sum = 0;
      double sumpow = 0;
      double mean = 0;
      double stddev = 0;

      for (int x=0; x<classes[i].size(); x++) {
        sum += classes[i][x][j];
      }
      mean = sum / classes[i].size();

      for (int x=0; x<classes[i].size(); x++) {
        sumpow += pow(classes[i][x][j]-mean, 2);
      }
      stddev = sqrt(sumpow / classes[i].size());

      model[i].push_back(mean);
      model[i].push_back(stddev);
    }
  }
}

string GNB::predict(vector<double> input)
{
  vector<double> probabilities(possible_labels.size());
  for (int i=0; i<possible_labels.size(); i++) {
    probabilities[i] = 1;
    for(int j=0; j<input.size(); j++) {
      double x = input[j];
      double mean = model[i][j*2];
      double stddev = model[i][j*2+1];
      double var = pow(stddev, 2);
      double exponent = exp(-pow((x-mean), 2) / (2 * var));
      double probability = 1 / (stddev * sqrt(2*M_PI)) * exponent;
      probabilities[i] *= probability;
    }
  }
  auto max = max_element(probabilities.begin(), probabilities.end());
	return this->possible_labels[std::distance(probabilities.begin(), max)];

}
