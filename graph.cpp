#include "graph.hpp"
///////////////////////////////////////////////////////////////////////////////
//                                  DISTANCE                                 //
///////////////////////////////////////////////////////////////////////////////
class Distance {
public:
  //! Default Constructor
  Distance() : Distance(0, false) {}

  Distance(const double d) : Distance(d, false) {}

  Distance(const double d, const bool b) : distance(d), isInf(b) {}

  //! Destructor
  virtual ~Distance() noexcept {}

  double distance;
  bool isInf;
};

bool operator<(const Distance &a, const Distance &b);
bool operator<(const Distance &a, const Distance &b) {
  bool res = true;
  if (!a.isInf && !b.isInf) {
    res = a < b;
  } else if (!a.isInf && b.isInf) {
    res = true;
  } else if (a.isInf && !b.isInf) {
    res = false;
  } else {
    res = false; // inf is not less than inf
  }
  return res;
}

Distance operator+(const Distance &lhs, const Distance &rhs);
Distance operator+(const Distance &lhs, const Distance &rhs) {
  Distance result;
  if (lhs.isInf || rhs.isInf) {
    result = {0.0, true};
  } else {
    result = {lhs.distance + rhs.distance, false};
  }
  return result;
}

///////////////////////////////////////////////////////////////////////////////
//                                   GRAPH                                    //
///////////////////////////////////////////////////////////////////////////////

Graph::Graph() : _nodes({{}}) {}

Graph::Graph(const std::string &filename) {
  std::ifstream infile;
  infile.open(filename);
  if (infile.is_open()) {
    // ignore NODES: line
    infile.ignore(256, '\n');
    while (infile.peek() != 'E') {
      NodeIndex idx;
      double x, y;
      infile >> idx >> x >> y;
      NodePtr n = std::make_shared<Node>(x, y);
      _nodes.insert(std::make_pair(idx, n));
    }
    // skip EDGES: line
    infile.ignore(256, '\n');
    while (infile.good()) {
      NodeIndex a, b;
      infile >> a >> b;
      _nodes.at(a)->addNeighbour(_nodes.at(b));
      _nodes.at(b)->addNeighbour(_nodes.at(a));
    }
  } else {
    throw std::runtime_error("Could not read " + filename + "\n");
  }
}

Graph::~Graph() noexcept {}

Graph &Graph::operator=(const Graph &rhs) {
  // Check for self-assignment!
  if (this == &rhs)
    return *this;
  for (auto &pair : rhs._nodes) {
    // deep copy Node
    NodePtr n = std::make_shared<Node>(*(pair.second));
    _nodes.insert(std::make_pair(pair.first, n));
  }

  return *this;
}

DistanceResult Graph::_edgeLength(const NodeIndex i, const NodeIndex j) {
  return _nodes.at(i)->getDistancetoNeighbour(_nodes.at(j));
}

double Graph::edgeLength(const NodeIndex i, const NodeIndex j) {
  return _edgeLength(i, j).distance;
}

double Graph::pathLength(const std::vector<NodeIndex> &path) {
  DistanceResult pathLengthResult(true, 0.0);
  for (auto it = path.begin(); it != path.end() - 1; ++it) {
    DistanceResult dr =
        _nodes.at(*it)->getDistancetoNeighbour(_nodes.at(*(it + 1)));
    if (!dr.valid) {
      pathLengthResult.valid = false;
      pathLengthResult.distance = -1;
    }
    if (pathLengthResult.valid) {
      pathLengthResult.distance += dr.distance;
    }
  }
  return pathLengthResult.distance;
}

std::vector<NodeIndex> Graph::navi(const NodeIndex i, const NodeIndex j) {
  // Initialisierung
  std::map<NodeIndex, Distance> distances;
  std::map<NodeIndex, std::pair<NodeIndex, NodePtr>> predecessors;
  std::map<NodeIndex, bool> visited;
  for (auto it = _nodes.begin(); it != _nodes.end(); ++it) {
    NodeIndex idx = it->first;
    NodePtr currentNode = it->second;
    if (idx != i) {
      distances.emplace(std::make_pair(idx, Distance(0.0, true)));
    } else {
      distances.emplace(std::make_pair(i, Distance(0.0, false)));
    }
    predecessors.emplace(idx, std::make_pair(0, nullptr));
    visited.emplace(idx, false);
  }
  // Initialisierung abgeschlossen

  // while not all nodes have been visited
  while (!std::all_of(
      visited.cbegin(), visited.cend(),
      [](const std::pair<NodeIndex, bool> &v) { return v.second; })) {
    auto u = std::min(distances.begin(), distances.end(), [](auto &a, auto &b) {
      return a->second < b->second;
    }); // use < Operator for Distance class

    // assert(!u.isInf, "No u with distance less than inf found!");

    // Dijkstra Algorithm main part
    NodeIndex u_idx = u->first;
    visited.at(u_idx) = true;
    for (auto v = _neighbours.at(u_idx).begin();
         v != _neighbours.at(u_idx).end(); ++v) {
      if (!visited.at(*v)) {
        // distance of u to start is valid because it's the minimum
        Distance alternative = distances.at(u_idx) + edgeLength(u_idx, *v);
        if (alternative < distances.at(*v)) {

          distances.at(*v) = alternative;
          predecessors.at(*v) = std::make_pair(u_idx, _nodes.at(u_idx));
        }
      }
    }
  }
  // Dijkstra is done

  // Build the path
  std::vector<NodeIndex> path = {j};
  auto u = predecessors.at(j);
  while (predecessors.at(u.first).second != nullptr) {
    u = predecessors.at(u.first);
    path.push_back(u.first);
  }
  // Path building done

  std::reverse(std::begin(path), std::end(path));

  return path;
}
