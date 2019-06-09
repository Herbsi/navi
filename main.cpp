#include "graph.hpp"
#include <iostream>
#include <vector>

using namespace std;

int main() {

  std::string filename = "Map.txt";
  Graph graph(filename);

  auto path1 = graph.navi(54, 56);
  // auto path2 = graph.navi(32, 9);
  // auto path3 = graph.navi(23, 8);

  std::cout << "Shortest Path between 54 & 56:\n";
  for (auto &node : path1) {
    std::cout << node << " ";
  }
  std::cout << "\n";

  // std::cout << "Shortest Path between 32 & 9:\n";
  // for (auto &node : path2) {
  //   std::cout << node << " ";
  // }
  // std::cout << "\n";

  // std::cout << "Shortest Path between 23 & 8:\n";
  // for (auto &node : path3) {
  //   std::cout << node << " ";
  // }
  // std::cout << "\n" << endl;

  return 0;
}
