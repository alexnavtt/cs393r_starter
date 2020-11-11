#include <iostream>
#include <queue>
#include <map>

using std::cout;
using std::endl;

struct Node
{
    double cost;
    int index;
    char key;
    // sort priority queue by cost
    bool operator<(const Node& rhs) const
    {
      // > for lowest cost sorting like how we want
      return cost > rhs.cost;
    }
};

int main()
{
  // How to work with a map
  std::map<char, Node> nav_map;
  Node n1{7,1,'a'};
  Node n2{3,2,'b'};
  nav_map[n1.key] = n1;
  nav_map[n2.key] = n2;
  nav_map['c'] = {8,3,'c'};
  cout << "Cost of the node with key b: " << nav_map['b'].cost << endl;

  std::priority_queue<Node> frontier;
  // 3 different ways to put things in a priority queue
  frontier.push(n1);
  frontier.push(n2);
  frontier.push(Node{8,3,'c'});
  cout << "Index of the node with the lowest cost: " << frontier.top().index << endl;

  cout << "Print the keys from lowest to highest cost: ";
  while(!frontier.empty())
  {
    Node first_node = frontier.top();
    cout << first_node.key << " ";
    frontier.pop();
  }
  cout << "\n";
  return 0;
}