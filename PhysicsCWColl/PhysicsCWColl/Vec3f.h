/*
// A basic class to help with the maths.
// Holds information about 3D vectors.
// Author:
// Jordan S. Gray
// 40087220
// Edinburgh Napier University
// BSc Games Development
// Year 3
// graybostephano@gmail.com
//
// Credit to:
// www.stackoverflow.com for samples of vector class.
*/

#ifndef VEC3F_H_INCLUDED
#define VEC3F_H_INCLUDED

#include <iostream>

//-----------------------------------------
//-----------------------------------------

//Basic Vec3 class to help with the maths.
class Vec3f {
private:
	float v[3];	//An array of floating points, x, y, z.
public:
	Vec3f();	//Basic constructor
	Vec3f(float x, float y, float z);	//Constructor takes in x, y, and z co-ordinates

	float &operator[](int index);
	float operator[](int index) const;

	Vec3f operator*(float scale) const;
	Vec3f operator/(float scale) const;
	Vec3f operator+(const Vec3f &other) const;
	Vec3f operator-(const Vec3f &other) const;
	Vec3f operator-() const;

	const Vec3f &operator*=(float scale);
	const Vec3f &operator/=(float scale);
	const Vec3f &operator+=(const Vec3f &other);
	const Vec3f &operator-=(const Vec3f &other);

	float magnitude() const;
	float magnitudeSquared() const;
	Vec3f normalize() const;
	float dot(const Vec3f &other) const;
	Vec3f cross(const Vec3f &other) const;
};

Vec3f operator*(float scale, const Vec3f &v);
std::ostream &operator<<(std::ostream &output, const Vec3f &v);



#endif