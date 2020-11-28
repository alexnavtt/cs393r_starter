#include <iostream>
#include <map>
#include <navigation/simple_queue.h>

using std::cout;
using std::endl;
using std::string;

struct Node
{
  float priority;
  int cost;
  string key;
};

int main()
{
  Node n1{7.0, 1, "a-key"};
  Node n2{3.0, 2, "b-key"};

  // How to work with a map
  std::map<string, Node> nav_map;

  nav_map[n1.key] = n1;
  nav_map[n2.key] = n2;
  nav_map["c-key"] = {9.0, 3, "c-key"};
  cout << "Priority of the node b: " << nav_map["b-key"].priority << endl;
  // How can you check whether a element exist in a map?
  cout << (nav_map.find("fake-key") != nav_map.end()) << endl;
  
  SimpleQueue<string, float> frontier;
  frontier.Push(n1.key, n1.priority);
  frontier.Push(n2.key, n2.priority);
  frontier.Push("b-key", nav_map["b-key"].priority);
  
  // Should print b since it has the lowest priority
  string key_1 = frontier.Pop();
  // Now update the priority on c to be lower than a
  nav_map["c-key"].priority = 5.0;
  frontier.Push(nav_map["c-key"].key, nav_map["c-key"].priority);
  // Should print c since b is gone and c now has the lowest priority
  string key_2 = frontier.Pop();
  // Remove a, should make the queue empty
  frontier.Pop();
  cout << key_1 << " " << key_2 << " " << frontier.Empty() << endl;

  // Joydeep's example code:
  // Construct the priority queue, to use uint64_t to represent node ID, and float as the priority type.
  SimpleQueue<uint64_t, float> queue;
  // Insert the node with ID 42 with priority 2.34 into the queue:
  queue.Push(42, 2.3);
  // Insert the node with ID 43 with priority 5.1 into the queue:
  queue.Push(43, 2.2);
  // Update the node with ID 42 with priority 2.1 into the queue:
  queue.Push(42, 2.1);
  // Get the priority element, this will return ID 42, since it has the better priority (2.1 < 2.2)
  uint64_t next_node = queue.Pop();
  cout << next_node << endl;

  return 0;
}