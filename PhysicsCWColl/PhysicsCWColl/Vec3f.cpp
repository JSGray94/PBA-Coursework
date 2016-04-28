/*
// See header file for detailed comments.
*/


//Declare any header files or libs being used.
#include "Vec3f.h"
#include <math.h>
//Declare namespaces.
using namespace std;

//-------------------------------------------------
//-------------------------------------------------

//Basic empty constructor.
Vec3f::Vec3f() {

}

//Constructor takes in x, y and z co-ordinates.
//Sets x, y and z to a position in the array v[]
Vec3f::Vec3f(float x, float y, float z) {
	v[0] = x;
	v[1] = y;
	v[2] = z;
}

//Returns the inputted index of the array
float &Vec3f::operator[](int index) {
	return v[index];
}

float Vec3f::operator[](int index) const {
	return v[index];
}

//Scales each of the values up in the co-ordinates by inputed floating point value.
Vec3f Vec3f::operator*(float scale) const {
	return Vec3f(v[0] * scale, v[1] * scale, v[2] * scale);
}

//Scales each of the values down in the co-ordinates by inputed floating point value.
Vec3f Vec3f::operator/(float scale) const {
	return Vec3f(v[0] / scale, v[1] / scale, v[2] / scale);
}

//Adds two vectors together
Vec3f Vec3f::operator+(const Vec3f &other) const {
	return Vec3f(v[0] + other.v[0], v[1] + other.v[1], v[2] + other.v[2]);
}

//Subtracts one vector from another
Vec3f Vec3f::operator-(const Vec3f &other) const {
	return Vec3f(v[0] - other.v[0], v[1] - other.v[1], v[2] - other.v[2]);
}

//Inverses the vector
Vec3f Vec3f::operator-() const {
	return Vec3f(-v[0], -v[1], -v[2]);
}

//Scales each vector point by itself upwards.
const Vec3f &Vec3f::operator*=(float scale) {
	v[0] *= scale;
	v[1] *= scale;
	v[2] *= scale;
	return *this;
}

//Scales each vector point by itself downwards
const Vec3f &Vec3f::operator/=(float scale) {
	v[0] /= scale;
	v[1] /= scale;
	v[2] /= scale;
	return *this;
}

//Adds each vector point to itself
const Vec3f &Vec3f::operator+=(const Vec3f &other) {
	v[0] += other.v[0];
	v[1] += other.v[1];
	v[2] += other.v[2];
	return *this;
}

//Subtracts each vector point from itself.
const Vec3f &Vec3f::operator-=(const Vec3f &other) {
	v[0] -= other.v[0];
	v[1] -= other.v[1];
	v[2] -= other.v[2];
	return *this;
}

//Gets the magnitude of the vector
float Vec3f::magnitude() const {
	return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

//Returns magnitude squared of each vector
float Vec3f::magnitudeSquared() const {
	return v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
}

//Normalizes teh vector.
Vec3f Vec3f::normalize() const {
	float m = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
	return Vec3f(v[0] / m, v[1] / m, v[2] / m);
}

//Finds dot product of vector.
float Vec3f::dot(const Vec3f &other) const {
	return v[0] * other.v[0] + v[1] * other.v[1] + v[2] * other.v[2];
}

//Finds cross product between vectors.
Vec3f Vec3f::cross(const Vec3f &other) const {
	return Vec3f(v[1] * other.v[2] - v[2] * other.v[1],
		v[2] * other.v[0] - v[0] * other.v[2],
		v[0] * other.v[1] - v[1] * other.v[0]);
}

//Takes in a float and vector and scales the vector by the scalar.
Vec3f operator*(float scale, const Vec3f &v) {
	return v * scale;
}

//Prints the vector positions/values.
ostream &operator<<(ostream &output, const Vec3f &v) {
	cout << '(' << v[0] << ", " << v[1] << ", " << v[2] << ')';
	return output;
}
