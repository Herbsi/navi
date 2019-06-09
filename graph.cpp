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
    NodePtr n = std::make_shared<Node>(*(std::get<1>(pair)));
    _nodes.insert(std::make_pair(std::get<0>(pair), n));
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

std::vector<NodeIndex> navi(const NodeIndex i, const NodeIndex j) {}
