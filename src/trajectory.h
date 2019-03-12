#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <string>
#include <vector>
#include <map>
#include <iostream>

using std::map;
using std::string;
using std::vector;
using std::ostream;

template <class T>
class Trajectory
{
public:
  Trajectory();
  Trajectory(string generated_by, vector<T>& path);
  ~Trajectory();

  friend ostream & operator << (ostream &out, const Trajectory &t) {
      out << "trajectory: " << t.generated_by << " -- total: " << t.total_cost;
      for (const auto& ic : t.individual_costs) {
          out << ", " << ic.first << "=" << ic.second;
      }
      return out;
  }

  string generated_by;
  double total_cost;
  map<string, double> individual_costs;
  vector<T> path;
};

template <class T>
Trajectory<T>::Trajectory() {}

template <class T>
Trajectory<T>::Trajectory(string generated_by, vector<T>& path):generated_by(generated_by), path(path) {}

template <class T>
Trajectory<T>::~Trajectory() {}

#endif // TRAJECTORY_H