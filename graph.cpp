#include "graph.hpp"

///////////////////////////////////////////////////////////////////////////////
//                                   GRAPH                                    //
///////////////////////////////////////////////////////////////////////////////

Graph::Graph() : _nodes({{}}), _neighbours({{}}) {}

Graph::Graph(const std::string &filename) {
  std::ifstream infile;
  infile.open(filename);
  if (infile.is_open()) {
    infile.ignore(256, '\n'); // ignore NODES: line
    NodeIndex idx;
    double x, y;
    while (infile >> idx >> x >> y) {
      NodePtr n = std::make_shared<Node>(x, y);
      _nodes.insert(std::make_pair(idx, n));
      _neighbours.insert(std::make_pair(idx, std::vector<NodeIndex>({})));
    }
    infile.clear();

    infile.ignore(256, '\n'); // skip EDGES: line
    NodeIndex a, b;
    while (infile >> a >> b) {
      try {
        _nodes.at(a)->addNeighbour(_nodes.at(b));
        _nodes.at(b)->addNeighbour(_nodes.at(a));
        _neighbours.at(a).push_back(b);
        _neighbours.at(b).push_back(a);
      } catch (std::out_of_range &) {
        std::cerr << "Invalid node index (at least 1): " << a << " " << b
                  << std::endl;
        throw;
      }
    }
  } else {
    throw std::runtime_error("Could not read " + filename + "\n");
  }
}

DistanceResult Graph::_edgeLength(const NodeIndex i, const NodeIndex j) {
  try {
    return _nodes.at(i)->getDistancetoNeighbour(_nodes.at(j));
  } catch (std::out_of_range &) {
    std::cerr << "Invalid indices (at least 1): " << i << " " << j << std::endl;
    throw;
  }
}

double Graph::edgeLength(const NodeIndex i, const NodeIndex j) {
  return _edgeLength(i, j).distance;
}

double Graph::pathLength(const Path &path) {
  DistanceResult pathLengthResult(true, 0.0);
  for (auto it = path.cbegin(); it != path.cend() - 1; ++it) {
    try {
      const DistanceResult dr =
          _nodes.at(*it)->getDistancetoNeighbour(_nodes.at(*(it + 1)));
      if (!dr.valid) {
        pathLengthResult.valid =
            false; // if any edge was invalid, stop adding to total distance
        pathLengthResult.distance = -1;
      }
      if (pathLengthResult.valid) {
        pathLengthResult.distance += dr.distance;
      }
    } catch (std::out_of_range &) {
      std::cerr << "Invalid node " << *it << std::endl;
      throw;
    }
  }
  return pathLengthResult.distance;
}

Path Graph::navi(const NodeIndex i, const NodeIndex j) {

  DNodeContainer dnc = _initialise(i);
  _dijkstra(dnc);
  return _builtPath(j, dnc.predecessors);
}

std::vector<NodeWithAngle> Graph::naviWithAngle(const NodeIndex i,
                                                const NodeIndex j) {
  auto path = navi(i, j);
  return _builtPathWithAngle(path);
}

DNodeContainer Graph::_initialise(const NodeIndex i) {
  std::map<NodeIndex, Distance> distances;
  std::map<NodeIndex, IndexedNode> predecessors;
  std::map<NodeIndex, bool> visited;

  for (auto it = _nodes.cbegin(); it != _nodes.cend(); ++it) {
    NodeIndex idx = it->first;
    NodePtr currentNode = it->second;
    distances.emplace(std::make_pair(idx, Distance(0.0, true)));
    predecessors.emplace(idx, std::make_pair(0, nullptr));
    visited.emplace(idx, false);
    if (idx == i)
      distances.at(idx).isInf = false;
  }

  return {distances, predecessors, visited};
}

Graph &Graph::_dijkstra(DNodeContainer &dnc) {

  // while not all nodes have been visited
  while (!std::all_of(dnc.visited.begin(), dnc.visited.end(),
                      [](auto &v) { return v.second; })) {

    // get element that is currently the least distance away from start point
    auto u = std::min_element(dnc.distances.begin(), dnc.distances.end(),
                              [](const auto &a, const auto &b) {
                                return a.second < b.second;
                              }); // use < Operator for Distance class

    if (u != dnc.distances.end()) {
      NodeIndex u_idx = u->first;
      dnc.visited.at(u_idx) = true;
      for (auto v = _neighbours.at(u_idx).cbegin();
           v != _neighbours.at(u_idx).cend(); ++v) {
        if (!dnc.visited.at(*v)) {
          // distance of u to start is valid because
          // it's the minimum
          Distance alternative =
              dnc.distances.at(u_idx) + edgeLength(u_idx, *v);
          if (alternative < dnc.distances.at(*v)) {

            dnc.distances.at(*v) = alternative;
            dnc.predecessors.at(*v) = std::make_pair(u_idx, _nodes.at(u_idx));
          }
        }
      }
      dnc.distances.erase(u);

    } else
      std::cerr << "Invalid minimum" << std::endl;
  }

  return *this;
}

std::vector<NodeIndex>
Graph::_builtPath(const NodeIndex endpoint,
                  const std::map<NodeIndex, IndexedNode> &predecessors) {

  std::vector<NodeIndex> path = {endpoint};
  path.reserve(predecessors.size() - 1);
  auto u = predecessors.at(endpoint);
  path.emplace_back(u.first);
  // start point is determined by not having a valid predecessor pointer
  while (predecessors.at(u.first).second != nullptr) {
    u = predecessors.at(u.first);
    path.emplace_back(u.first);
  }

  std::reverse(std::begin(path), std::end(path));

  return path;
}

std::vector<NodeWithAngle>
Graph::_builtPathWithAngle(const std::vector<NodeIndex> &path) {

  std::vector<NodeWithAngle> pathWithAngle;
  pathWithAngle.reserve(path.size());
  pathWithAngle.emplace_back(*path.begin(), Angle(0.0, true, true));
  for (auto it = path.begin() + 1; it != path.end() - 1; ++it) {
    auto prevNode = _nodes.at(*(it - 1));
    auto currentNode = _nodes.at(*it);
    auto nextNode = _nodes.at(*(it + 1));
    pathWithAngle.emplace_back(*it, angle(prevNode, currentNode, nextNode));
  }
  pathWithAngle.emplace_back(*(path.end() - 1), Angle(0.0, true, true));
  return pathWithAngle;
}

///////////////////////////////////////////////////////////////////////////////
//                                  Distance                                 //
///////////////////////////////////////////////////////////////////////////////

bool operator<(const Distance &a, const Distance &b) {
  bool res = true;
  if (!a.isInf && !b.isInf) {

    res = a.distance < b.distance;
  } else if (!a.isInf && b.isInf) {
    res = true;
  } else if (a.isInf && !b.isInf) {
    res = false;
  } else {
    res = false; // inf is not less than inf
  }
  return res;
}

Distance operator+(const Distance &lhs, const Distance &rhs) {
  Distance result;
  if (lhs.isInf || rhs.isInf) {
    result = {0.0, true};
  } else {
    result = {lhs.distance + rhs.distance, false};
  }
  return result;
}
