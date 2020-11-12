#include <stdio.h>
#include <iostream>
#include <cmath>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "shared/math/math_util.h"

using std::cout;
using std::endl;
using std::sqrt;
using std::sin;
using std::cos;

using Eigen::Vector2f;
using Eigen::Matrix2f;
using Eigen::Rotation2Df;

using math_util::AngleDiff;

void DemoBasics() {
  cout << "Basic initialization" << endl;
  cout << "Initialize a 2D vector v1." << endl;
  Vector2f v1(1.0, 2.0);

  cout << "Read elements of the 3D vector:" << endl
       << "v1.x = " << v1.x() << endl
       << "v1.y = " << v1.y() << endl;

  // Mark: norm test
  cout << "Get norm of a vector:" << endl;
  float v1norm = v1.norm();
  cout << v1norm << endl;
  // L1 norm
  cout << "Get L1 norm of a vector:" << endl;
  float v1normL1 = (-1*v1).lpNorm<1>();
  cout << v1normL1 << endl;

  // Mark: can you just subtract 2 vectors?
  Vector2f vsub(2.0, 1.0);
  vsub = v1 - vsub;
  cout << "[1, 2] - [2, 1] = [-1, 1]?" << endl
       << "Result: [" << vsub.x() << ", " << vsub.y() << "]" << endl;

  cout << "Write 10 to the x coordinate of v1:" << endl;
  v1.x() = 10.0;
  cout << "v1.x = " << v1.x() << endl;

  cout << "Print the vector to stdout:\n" << v1 << endl;

  cout << "Can you just multiply a vector?\n" << v1*2 << endl;

  cout << "Initialize a 2x2 matrix m1." << endl;
  Matrix2f m1;
  m1  << 0, 2,
         3, 0;
  cout << "m1 = " << endl << m1 << endl;

  cout << "Multiply matrix times vector." << endl;
  Vector2f v2 = m1 * v1;
  cout << "Resulting vector:\n" << v2 << endl;

  cout << "Test: " << (v1-v2).norm() << endl;
}

void DemoRotations() {
  cout << "Rotations demonstration" << endl;
  // Note that pi/4 radians is 45 degrees.
  float angle1 = M_PI / 4.0;
  cout << "angle1 = " << angle1 << " radians = "
       << angle1 / M_PI * 180.0 << " degrees." << endl;

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

void otherTests() {
  // angle_diff = AngleDiff(curr_angle, prev_angle)
  float angle_diff_1 = AngleDiff(6.2, 0.1); // should be -0.183
  float angle_diff_2 = AngleDiff(0.1, 6.2); // should be 0.183
  float angle_diff_3 = AngleDiff(1.7, 4.2); // should be -2.5
  float angle_diff_4 = AngleDiff(5.2, 2.9); // should be 2.3

  printf("Wrap angle results: %f,%f,%f,%f\n", angle_diff_1, angle_diff_2, angle_diff_3, angle_diff_4);
}

int main() {
  cout << "Basics: Vectors and Matrices.\n";
  DemoBasics();

  cout << "\n\n\nDifferent representations of rotation.\n";
  DemoRotations();

  cout << "non-eigen tests" << endl;
  otherTests();

  return 0;
}
