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
      const auto& start = t.path[0];
      const auto& end = t.path[t.path.size()-1];
      out << "trajectory: " << t.generated_by << " debug<" << t.debug << "> s<" << start.s << "," << start.vs << "," << start.as << "> d<"
        << start.d << "," << start.vd << "," << start.ad << "> to s<" << end.s << "," << end.vs << "," << end.as << "> d<"
        << end.d << "," << end.vd << "," << end.ad << "> -- total: " << t.total_cost;
      for (const auto& ic : t.individual_costs) {
          out << ", " << ic.first << "=" << ic.second;
      }
      return out;
  }

  string generated_by;
  string debug;
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