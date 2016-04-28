/*
Minimal set of math classes:
Vector3, Matrix4, and Quaternion with the all important
overloaded math operators - so we can write clearer more
understandable code :)

Part of the Uncomplicated Game Technology Tutorials

www.napier.ac.uk/games/

b.kenwright@napier.ac.uk
*/

#pragma once


// Maths - e.g., sin, sqrt
#include <math.h>
#include <cmath>

inline
float Sign(float val)
{
	if (val<0) return -1;
	return 1;
};


inline
float Clamp(float v, float min, float max)
{
	float res = v;
	res = v>max ? max : v;
	res = v<min ? min : v;
	return res;
};



// -------------------------------------------------------------------------

// Minimal vector class of 3 floats and overloaded math operators

class Vector3
{
public:
	float x, y, z;

	Vector3(float x, float y, float z)			{ this->x = x; this->y = y; this->z = z; }
	Vector3()									{}

	void	operator+= (const Vector3 &v)		{ x += v.x;   y += v.y;   z += v.z; }
	void	operator-= (const Vector3 &v)		{ x -= v.x;   y -= v.y;   z -= v.z; }
	void	operator*= (const float    s)		{ x *= s;     y *= s;     z *= s; }
	Vector3 operator/  (const float &v)	  const	{ return Vector3(x / v, y / v, z / v); }
	Vector3 operator-  (const Vector3 &v) const	{ return Vector3(x - v.x, y - v.y, z - v.z); }
	Vector3 operator+  (const Vector3 &v) const { return Vector3(x + v.x, y + v.y, z + v.z); }
	Vector3 operator*  (const float &v)   const	{ return Vector3(x*v, y*v, z*v); }
	Vector3 operator-  ()				  const	{ return Vector3(-x, -y, -z); }

	__forceinline static Vector3  Cross(const Vector3 &vA, const Vector3& vB)	{ return Vector3(vA.y*vB.z - vA.z*vB.y, vA.z*vB.x - vA.x*vB.z, vA.x*vB.y - vA.y*vB.x); }
	__forceinline static float	Dot(const Vector3 &vA, const Vector3& vB)	{ return vA.x*vB.x + vA.y*vB.y + vA.z*vB.z; }
	__forceinline static Vector3 Normalize(const Vector3& v)
	{
		const float l = v.Length();
		return Vector3(v.x / l, v.y / l, v.z / l);
	}
	float	Length()	 const					{ return sqrt(x*x + y*y + z*z); }
	float	LengthSq()	 const					{ return (x*x + y*y + z*z); }

	__forceinline static  float Length(const Vector3& v)
	{
		return v.Length();
	}

}; // End Vector3(..)



// -------------------------------------------------------------------------
//
// Matrix4
//
// -------------------------------------------------------------------------


/*
* Column-major 4x4 matrix
*
* Layout:
*			0  4  8  12
*			1  5  9  13
*			2  6  10 14
*			3  7  11 15
*
*  3x3 Rotation Matrix Indices
*			0  4  8
*			1  5  9
*			2  6  10
*
*  3x1 Translation Indices
*			12
*			13
*			14
*
*/
class Matrix4
{
public:
	float m[16];

	Matrix4() { }

	Matrix4(float _00, float _10, float _20, float _30,
		float _01, float _11, float _21, float _31,
		float _02, float _12, float _22, float _32,
		float _03, float _13, float _23, float _33)
	{
		m[0] = _00;	m[1] = _10;	m[2] = _20;	m[3] = _30;
		m[4] = _01;	m[5] = _11;	m[6] = _21;	m[7] = _31;
		m[8] = _02;	m[9] = _12;	m[10] = _22;	m[11] = _32;
		m[12] = _03;  m[13] = _13;	m[14] = _23;	m[15] = _33;
	}

	float&			operator[]	(int index)			{ return m[index]; }
	const float&	operator[]	(int index) const	{ return m[index]; }

	static Matrix4 Zero()
	{
		static Matrix4 zero(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
		return zero;
	}

	static Matrix4 Identity()
	{
		static Matrix4 identity(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);
		return identity;
	}

	static
		Matrix4 SetTranslation(const Vector3& translation)
	{
		Matrix4 mat = Matrix4::Identity();
		mat.m[12] = translation.x;
		mat.m[13] = translation.y;
		mat.m[14] = translation.z;
		return mat;
	}

	Vector3 GetTranslation() const
	{
		return Vector3(m[12], m[13], m[14]);
	}

	static
		Vector3 GetTranslation(const Matrix4& mat)
	{
		return Vector3(mat.m[12], mat.m[13], mat.m[14]);
	}

	static Matrix4 SetRotationAxis(const Vector3& axis, float angle)
	{
		// Page 466, Graphics Gems

		float s = sin(angle);
		float c = cos(angle);
		float t = 1 - c;

		Vector3 ax = Vector3::Normalize(axis);

		float x = ax.x;
		float y = ax.y;
		float z = ax.z;

		Matrix4 rotate(t*x*x + c, t*x*y - s*z, t*x*z + s*y, 0,
			t*y*x + s*z, t*y*y + c, t*y*z - s*x, 0,
			t*z*x - s*y, t*z*y + s*x, t*z*z + c, 0,
			0, 0, 0, 1);

		return rotate;
	}

	static
		Vector3 Transform(const Matrix4& mat, const Vector3& p)
	{
		return Vector3(
			mat[0] * p.x +
			mat[4] * p.y +
			mat[8] * p.z,

			mat[1] * p.x +
			mat[5] * p.y +
			mat[9] * p.z,

			mat[2] * p.x +
			mat[6] * p.y +
			mat[10] * p.z
			) + Vector3(mat[12], mat[13], mat[14]);
	}

	static
		Matrix4 Transpose(const Matrix4& mat)
	{
		Matrix4 retMat(mat);
		for (int i = 0; i < 4; i++)
		{
			for (int i2 = 0; i2 < 4; i2++)
			{
				retMat[i2 * 4 + i] = mat[i * 4 + i2];
			}
		}

		return retMat;
	}// End Transpose(..)

}; // End Matrix4(..)




inline
Matrix4 operator*(const Matrix4& rhs, const Matrix4& lhs)
{
	Matrix4 result;

	result[0] = lhs[0] * rhs[0] + lhs[4] * rhs[1] + lhs[8] * rhs[2] + lhs[12] * rhs[3];
	result[1] = lhs[1] * rhs[0] + lhs[5] * rhs[1] + lhs[9] * rhs[2] + lhs[13] * rhs[3];
	result[2] = lhs[2] * rhs[0] + lhs[6] * rhs[1] + lhs[10] * rhs[2] + lhs[14] * rhs[3];
	result[3] = lhs[3] * rhs[0] + lhs[7] * rhs[1] + lhs[11] * rhs[2] + lhs[15] * rhs[3];

	result[4] = lhs[0] * rhs[4] + lhs[4] * rhs[5] + lhs[8] * rhs[6] + lhs[12] * rhs[7];
	result[5] = lhs[1] * rhs[4] + lhs[5] * rhs[5] + lhs[9] * rhs[6] + lhs[13] * rhs[7];
	result[6] = lhs[2] * rhs[4] + lhs[6] * rhs[5] + lhs[10] * rhs[6] + lhs[14] * rhs[7];
	result[7] = lhs[3] * rhs[4] + lhs[7] * rhs[5] + lhs[11] * rhs[6] + lhs[15] * rhs[7];

	result[8] = lhs[0] * rhs[8] + lhs[4] * rhs[9] + lhs[8] * rhs[10] + lhs[12] * rhs[11];
	result[9] = lhs[1] * rhs[8] + lhs[5] * rhs[9] + lhs[9] * rhs[10] + lhs[13] * rhs[11];
	result[10] = lhs[2] * rhs[8] + lhs[6] * rhs[9] + lhs[10] * rhs[10] + lhs[14] * rhs[11];
	result[11] = lhs[3] * rhs[8] + lhs[7] * rhs[9] + lhs[11] * rhs[10] + lhs[15] * rhs[11];

	result[12] = lhs[0] * rhs[12] + lhs[4] * rhs[13] + lhs[8] * rhs[14] + lhs[12] * rhs[15];
	result[13] = lhs[1] * rhs[12] + lhs[5] * rhs[13] + lhs[9] * rhs[14] + lhs[13] * rhs[15];
	result[14] = lhs[2] * rhs[12] + lhs[6] * rhs[13] + lhs[10] * rhs[14] + lhs[14] * rhs[15];
	result[15] = lhs[3] * rhs[12] + lhs[7] * rhs[13] + lhs[11] * rhs[14] + lhs[15] * rhs[15];

	return result;
}



// -------------------------------------------------------------------------
//
// Quaternions
//
// -------------------------------------------------------------------------


class Quaternion
{
public:
	float x, y, z, w;

	Quaternion(float xx = 0, float yy = 0, float zz = 0, float ww = 1) : x(xx), y(yy), z(zz), w(ww) { };
	Quaternion(const Quaternion& rhs) : x(rhs.x), y(rhs.y), z(rhs.z), w(rhs.w) { };

	Quaternion& operator=(const Quaternion& rhs)
	{
		if (this != &rhs)
		{
			x = rhs.x;
			y = rhs.y;
			z = rhs.z;
			w = rhs.w;
		}

		return *this;
	}

	Quaternion& operator*=(const Quaternion& rhs)
	{
		float tmpx = w*rhs.x + x*rhs.w + z*rhs.y - y*rhs.z;
		float tmpy = w*rhs.y + y*rhs.w + x*rhs.z - z*rhs.x;
		float tmpz = w*rhs.z + z*rhs.w + y*rhs.x - x*rhs.y;
		float tmpw = w*rhs.w - x*rhs.x - y*rhs.y - z*rhs.z;

		x = tmpx;
		y = tmpy;
		z = tmpz;
		w = tmpw;

		return *this;
	}


	Quaternion& operator+=(const Quaternion& rhs)
	{
		float tmpw = w + rhs.w;
		float tmpx = x + rhs.x;
		float tmpy = y + rhs.y;
		float tmpz = z + rhs.z;

		x = tmpx;
		y = tmpy;
		z = tmpz;
		w = tmpw;

		return *this;
	}

	// convenience function for the identity quaternion
	static Quaternion Identity()
	{
		static Quaternion identity(0, 0, 0, 1);
		return identity;
	}

	static
		Quaternion Conjugate(const Quaternion& q)
	{
		return Quaternion(-q.x, -q.y, -q.z, q.w);
	}

	static
		float Length(const Quaternion& q)
	{
		return std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
	}

	static
		float LengthSq(const Quaternion& q)
	{
		return q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
	}

	static
		Quaternion Normalize(const Quaternion& q)
	{
		float magInv = 1.f / Length(q);

		return Quaternion(q.x*magInv, q.y*magInv, q.z*magInv, q.w*magInv);
	}

	static
		float Dot(const Quaternion& q1, const Quaternion& q2)
	{
		return ((q1.x*q2.x) + (q1.y*q2.y) + (q1.z*q2.z) + (q1.w*q2.w));
	}

	static
		Quaternion FromAxisAngle(const Vector3& v, float angle)
	{
		angle *= 0.5f;
		float sinAngle = std::sin(angle);

		Vector3 normVector = Vector3::Normalize(v);

		return Quaternion(normVector.x*sinAngle,
			normVector.y*sinAngle,
			normVector.z*sinAngle,
			std::cos(angle));
	}

	static
		float ToAxisAngle(const Quaternion& q, Vector3& v)
	{
		// The quaternion representing the rotation is
		//   q = cos(A/2)+sin(A/2)*(x*i+y*j+z*k)

		float sqrLength = q.x*q.x + q.y*q.y + q.z*q.z;
		if (sqrLength > 0.0f)
		{
			float invLength = 1.0f / std::sqrt(sqrLength);

			v.x = q.x*invLength;
			v.y = q.y*invLength;
			v.z = q.z*invLength;

			return 2.f*std::acos(q.w);
		}
		else
		{
			// angle is 0 (mod 2*pi), so any axis will do.
			v.x = 1.0f;
			v.y = 0.0f;
			v.z = 0.0f;

			return 0.f;
		}
	}

	static
		Matrix4 ToRotationMatrix(const Quaternion& q)
	{
		const float x2 = q.x*q.x;
		const float y2 = q.y*q.y;
		const float z2 = q.z*q.z;
		const float xy = q.x*q.y;
		const float xz = q.x*q.z;
		const float yz = q.y*q.z;
		const float wx = q.w*q.x;
		const float wy = q.w*q.y;
		const float wz = q.w*q.z;

		// filling up row-wise
		Matrix4 mat;
		mat[0] = 1.0f - 2.0f*(y2 + z2);		// [0][0]
		mat[4] = 2.0f*(xy - wz);				// [0][1]
		mat[8] = 2.0f*(xz + wy);				// [0][2]
		mat[12] = 0.f;						// [0][3]

		mat[1] = 2.0f*(xy + wz);				// [1][0]
		mat[5] = 1.0f - 2.0f*(x2 + z2);		// [1][1]
		mat[9] = 2.0f*(yz - wx);				// [1][2]
		mat[13] = 0.f;						// [1][3]

		mat[2] = 2.0f*(xz - wy);				// [2][0]
		mat[6] = 2.0f*(yz + wx);				// [2][1]
		mat[10] = 1.0f - 2.0f*(x2 + y2);		// [2][2]
		mat[14] = 0.f;						// [2][3]

		mat[3] = 0.f;						// [3][0]
		mat[7] = 0.f;						// [3][1]
		mat[11] = 0.f;						// [3][2]
		mat[15] = 1.f;						// [3][3]
		return mat;
	}

	static
		Quaternion FromRotationMatrix(const Matrix4& m)
	{
		Quaternion q(0, 0, 0, 1);

		float trace = m[0] + m[5] + m[10] + 1.f;

		float tolerance = 0.00001f;

		if (trace > tolerance)
		{
			float root = std::sqrt(trace);
			q.w = 0.5f*root;
			root = 0.5f / root;
			q.x = (m[6] - m[9]) * root;
			q.y = (m[8] - m[2]) * root;
			q.z = (m[1] - m[4]) * root;
		}
		else
		{
			if (m[0] > m[5] && m[0] > m[10])
			{
				float root = std::sqrt(m[0] - m[5] - m[10] + 1.f);
				q.x = 0.5f*root;
				root = 0.5f / root;
				q.y = (m[1] - m[4]) * root;
				q.z = (m[8] - m[2]) * root;
				q.w = (m[6] - m[9]) * root;
			}
			else if (m[5] > m[10])
			{
				float root = std::sqrt(m[5] - m[0] - m[10] + 1.f);
				q.y = 0.5f*root;
				root = 0.5f / root;
				q.x = (m[1] - m[4]) * root;
				q.z = (m[6] - m[9]) * root;
				q.w = (m[8] - m[2]) * root;
			}
			else
			{
				float root = std::sqrt(m[10] - m[0] - m[5] + 1.f);
				q.z = 0.5f*root;
				root = 0.5f / root;
				q.x = (m[8] - m[2]) * root;
				q.y = (m[6] - m[9]) * root;
				q.w = (m[1] - m[4]) * root;
			}
		}

		return q;
	}

	static
		Quaternion Slerp(float t, const Quaternion& p, const Quaternion& q)
	{
		Quaternion ret(0, 0, 0, 1);

		float cs = Quaternion::Dot(p, q);
		float angle = std::acos(cs);

		if (std::abs(angle) > 0.0f)
		{
			float sn = std::sin(angle);
			float invSn = 1.0f / sn;
			float tAngle = t*angle;
			float coeff0 = std::sin(angle - tAngle)*invSn;
			float coeff1 = std::sin(tAngle)*invSn;

			ret.x = coeff0*p.x + coeff1*q.x;
			ret.y = coeff0*p.y + coeff1*q.y;
			ret.z = coeff0*p.z + coeff1*q.z;
			ret.w = coeff0*p.w + coeff1*q.w;
		}
		else
		{
			ret.x = p.x;
			ret.y = p.y;
			ret.z = p.z;
			ret.w = p.w;
		}

		return ret;
	}// End Slerp(..)

}; // End Quaternion(...)


inline
Quaternion operator*(const Quaternion& q1, const Quaternion& q2)
{
	return Quaternion(q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y,
		q1.w*q2.y + q1.y*q2.w + q1.z*q2.x - q1.x*q2.z,
		q1.w*q2.z + q1.z*q2.w + q1.x*q2.y - q1.y*q2.x,
		q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z);
} // End operator*(..)


inline
Quaternion operator*(const Quaternion& q, float s)
{
	return Quaternion(q.x*s, q.y*s, q.z*s, q.w*s);
}// End operator*(..)

inline
Quaternion operator*(float s, const Quaternion& q)
{
	return Quaternion(q.x*s, q.y*s, q.z*s, q.w*s);
}// End operator*(..)


inline
Quaternion operator+(const Quaternion& q1, const Quaternion& q2)
{
	return Quaternion(q1.x + q2.x,
		q1.y + q2.y,
		q1.z + q2.z,
		q1.w + q2.w);
}// End operator*(..)


