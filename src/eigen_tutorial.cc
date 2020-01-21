#include <stdio.h>
#include <iostream>
#include <cmath>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

using std::cout;
using std::endl;
using std::sqrt;
using std::sin;
using std::cos;

using Eigen::Vector2f;
using Eigen::Matrix2f;
using Eigen::Matrix3f;
using Eigen::Vector3f;
using Eigen::Rotation2Df;

void DemoBasics() {
  cout << "Basic initialization" << endl;
  cout << "Initialize a vector v1." << endl;
  Vector3f v1(1.0, 2.0, 3.0);

  cout << "Read elements of the vector:" << endl
       << "v1.x = " << v1.x() << endl
       << "v1.y = " << v1.y() << endl
       << "v1.z = " << v1.z() << endl;

  cout << "Write 10 to the x coordinate of v1:" << endl;
  v1.x() = 10.0;
  cout << "v1.x = " << v1.x() << endl;

  cout << "Print the vector to stdout:\n" << v1 << endl;

  cout << "Initialize a matrix m1." << endl;
  Matrix3f m1;
  m1  << 2, 0, 0,
         0, 0, 4,
         0, 1, 0;

  cout << "Multiply matrix times vector." << endl;
  Vector3f v2 = m1 * v1;
  cout << "Resulting vector:\n" << v2 << endl;
}

void DemoRotations() {
  cout << "Rotations demonstration" << endl;
  float angle1 = M_PI / 4.0;

  cout << "Create a rotation" << endl;
  Rotation2Df r1(angle1);

  cout << "Apply that rotation to a vector" << endl;
  Vector2f v1(1.0, 0);
  Vector2f v2 = r1 * v1;
  cout << v2 << "\n";

  cout << "Convert rotation to a matrix" << endl;
  Matrix2f m1 = r1.toRotationMatrix();
  cout << "m1: \n" << m1 << "\n";
}

int main() {
  cout << "Basics: Vectors and Matrices.\n";
  DemoBasics();

  cout << "\n\n\nDifferent representations of rotation.\n";
  DemoRotations();

  return 0;
}
