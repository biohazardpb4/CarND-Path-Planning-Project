#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <string>
#include <vector>
#include <map>

using std::map;
using std::string;
using std::vector;

template <class T>
class Trajectory
{
public:
  Trajectory();
  Trajectory(vector<T>& path);
  ~Trajectory();

  string generated_by;
  double total_cost;
  map<string, double> individual_costs;
  vector<T> path;
};

template <class T>
Trajectory<T>::Trajectory() {}

template <class T>
Trajectory<T>::Trajectory(vector<T>& path):path(path) {}

template <class T>
Trajectory<T>::~Trajectory() {}

#endif // TRAJECTORY_H