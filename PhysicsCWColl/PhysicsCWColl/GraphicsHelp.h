/*
Uncomplicated Simulation Tutorial

Single self contained sample program - for educational purposes - it should
be straightforward to follow through, disect and understand, so that you are
able to expand and put into your own program

www.napier.ac.uk/games/

b.kenwright@napier.ac.uk

Details:  Basic set of helper functions for drawing simple
graphics - e.g., spheres, arrows, lines...
*/



#pragma once


//#pragma comment(lib, "OpenGL32.Lib")   // glVertex3, glClear, ..
//#pragma comment(lib, "glu32.lib")	   // gluLookAt, ..
//#pragma comment(lib, "GL/glut32.lib")  // also needs glut32.dll
#include "GL/glut.h"

#include <vector>		// vector list

#include "MathsHelp.h" // Vector3, Matrix4, and Quaternion


// Basic assert function to do some checking - if the check is violate
// i.e. false, then this line will cause the debugger to stop on the line
#define DBG_ASSERT(f) { if(!(f)){ __debugbreak(); }; }



inline
float lerp(float a, float b, float t)
{
	return a + t*(b - a);
}



// create a color ramp
inline
void ColorRamp(float t, float& r, float& g, float& b)
{
	if (t > 1) t = 1;

	const int ncolors = 7;
	float c[ncolors][3] =
	{
		{ 1.0, 0.0, 0.0, },
		{ 1.0, 0.5, 0.0, },
		{ 1.0, 1.0, 0.0, },
		{ 0.0, 1.0, 0.0, },
		{ 0.0, 1.0, 1.0, },
		{ 0.0, 0.0, 1.0, },
		{ 1.0, 0.0, 1.0, },
	};
	t = t * (ncolors - 1);
	int i = (int)t;
	float u = t - floor(t);
	b = lerp(c[i][0], c[i + 1][0], u);
	g = lerp(c[i][1], c[i + 1][1], u);
	r = lerp(c[i][2], c[i + 1][2], u);
}// End ColorRamp(..)




// -------------------------------------------------------------------------

// Basic Draw Functions (e.g., for lines, points )

// Takes three paramaters, i.e., the start and end position in 3D
// of the arrow position in the world, and the size of the arrow
inline
void DrawArrow(const Vector3& p0, const Vector3& p1, GLdouble D)
{
	double x = p1.x - p0.x;
	double y = p1.y - p0.y;
	double z = p1.z - p0.z;
	double L = sqrt(x*x + y*y + z*z);

	GLUquadricObj *quadObj;

	glPushMatrix();

	glTranslated(p0.x, p0.y, p0.z);

#define RADPERDEG 0.0174533

	if ((x != 0.) || (y != 0.)) {
		glRotated(atan2(y, x) / RADPERDEG, 0., 0., 1.);
		glRotated(atan2(sqrt(x*x + y*y), z) / RADPERDEG, 0., 1., 0.);
	}
	else if (z<0){
		glRotated(180, 1., 0., 0.);
	}

	glTranslatef(0, 0, L - 4 * D);

	quadObj = gluNewQuadric();
	gluQuadricDrawStyle(quadObj, GLU_FILL);
	gluQuadricNormals(quadObj, GLU_SMOOTH);
	gluCylinder(quadObj, 2 * D, 0.0, 4 * D, 32, 1);
	gluDeleteQuadric(quadObj);

	quadObj = gluNewQuadric();
	gluQuadricDrawStyle(quadObj, GLU_FILL);
	gluQuadricNormals(quadObj, GLU_SMOOTH);
	gluDisk(quadObj, 0.0, 2 * D, 32, 1);
	gluDeleteQuadric(quadObj);

	glTranslatef(0, 0, -L + 4 * D);

	quadObj = gluNewQuadric();
	gluQuadricDrawStyle(quadObj, GLU_FILL);
	gluQuadricNormals(quadObj, GLU_SMOOTH);
	gluCylinder(quadObj, D, D, L - 4 * D, 32, 1);
	gluDeleteQuadric(quadObj);

	quadObj = gluNewQuadric();
	gluQuadricDrawStyle(quadObj, GLU_FILL);
	gluQuadricNormals(quadObj, GLU_SMOOTH);
	gluDisk(quadObj, 0.0, D, 32, 1);
	gluDeleteQuadric(quadObj);

	glPopMatrix();

}// End DrawArrow(..)

// -------------------------------------------------------------------------

// For simple displaying of lines - takes two 3D positions
// start and end of the line in the 3D world
inline
void DrawLine3D(const Vector3& p0, const Vector3& p1)
{
	glPushMatrix();
	glBegin(GL_LINES);
	glColor3f(1, 0.0, 0.0); // Color RED
	glVertex3d(p0.x, p0.y, p0.z);
	glVertex3d(p1.x, p1.y, p1.z);
	glEnd();
	glPopMatrix();
}// End DrawLine(..)





// -------------------------------------------------------------------------

// For simple displaying of triangles
inline
void DrawTriangle(const Vector3& p0, const Vector3& p1, const Vector3& p2, float r = 0, float g = 0.6f, float b = 0, float a = 0.5f)
{
	Vector3 n = Vector3::Normalize(Vector3::Cross(p1 - p0, p2 - p0));
	glPushMatrix();
	//glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
	//glPolygonMode(GL_FRONT, GL_LINE);
	glBegin(GL_TRIANGLES);


	glColor4f(r, g, b, a); // Color Green

	glNormal3f(n.x, n.y, n.z);
	glVertex3d(p0.x, p0.y, p0.z);

	glNormal3f(n.x, n.y, n.z);
	glVertex3d(p1.x, p1.y, p1.z);

	glNormal3f(n.x, n.y, n.z);
	glVertex3d(p2.x, p2.y, p2.z);
	glEnd();
	glPopMatrix();
}// End DrawLine(..)





inline
void DrawTriangle(const Vector3& p0,
const Vector3& p1,
const Vector3& p2,
const Vector3& n0,
const Vector3& n1,
const Vector3& n2,
//const Vector3& uv0,
//const Vector3& uv1,
//const Vector3& uv2,
const Vector3& c0,
const Vector3& c1,
const Vector3& c2)
{
	//Vector3 n = Vector3::Normalize( Vector3::Cross( p1-p0, p2-p0 ) );
	glPushMatrix();

	//glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
	//glPolygonMode(GL_FRONT, GL_LINE);

	float a = 0.8f;
	glBegin(GL_TRIANGLES);
	glColor4f(c0.x, c0.y, c0.z, a); // colour

	//glTexCoord2f(uv0.x, uv0.y);
	glNormal3f(n0.x, n0.y, n0.z);
	glVertex3d(p0.x, p0.y, p0.z);

	glColor4f(c1.x, c1.y, c1.z, a); // colour
	//glTexCoord2f(uv1.x, uv1.y);
	glNormal3f(n1.x, n1.y, n1.z);
	glVertex3d(p1.x, p1.y, p1.z);

	glColor4f(c2.x, c2.y, c2.z, a); // colour
	//glTexCoord2f(uv2.x, uv2.y);
	glNormal3f(n2.x, n2.y, n2.z);
	glVertex3d(p2.x, p2.y, p2.z);
	glEnd();
	glPopMatrix();
}// End DrawLine(..)

// -------------------------------------------------------------------------

// Draw 3D spherical mesh in the environment
// paramaters should be self explanitory (i.e., position, radius and colour)
inline
void DrawSphere(const Vector3& p0, float radius, float r = 1, float g = 0, float b = 0)
{
	glPushMatrix();
	glColor3f(r, g, b);
	glTranslatef(p0.x, p0.y, p0.z);
	glutSolidSphere(radius, 10, 9);
	glPopMatrix();
}// End DrawSphere(..)


// -------------------------------------------------------------------------
//
// Basic draw solid cube function..
//
// -------------------------------------------------------------------------
inline
void DrawCube(const Vector3& pos, const Quaternion& rot, const Vector3& dims, float r = 1, float g = 0, float b = 0)
{
	glPushMatrix();
	glColor3f(r, g, b);


	float angle;
	Vector3 axis;
	angle = Quaternion::ToAxisAngle(rot, axis);

#define RADPERDEG 0.0174533



	glTranslatef(pos.x, pos.y, pos.z);

	glRotatef(angle / RADPERDEG, axis.x, axis.y, axis.z);


	glScalef(dims.x, dims.y, dims.z);


	glutSolidCube(1.0f);
	glPopMatrix();
}// End DrawCube(..)


// -------------------------------------------------------------------------

// -------------------------------------------------------------------------

// Draws a simple wireframe bounding box of size 2x2x2
inline
void DrawBoundingBox()
{
	glPushMatrix();
	int i, j;

	glColor4f(0.6f, 0.6f, 0.6f, 1);

	glBegin(GL_LINES);

	// front face
	for (i = -2; i <= 2; i++)
	{
		glVertex3f(i, -2, -2);
		glVertex3f(i, -2, 2);
	}
	for (j = -2; j <= 2; j++)
	{
		glVertex3f(-2, -2, j);
		glVertex3f(2, -2, j);
	}

	// back face
	for (i = -2; i <= 2; i++)
	{
		glVertex3f(i, 2, -2);
		glVertex3f(i, 2, 2);
	}
	for (j = -2; j <= 2; j++)
	{
		glVertex3f(-2, 2, j);
		glVertex3f(2, 2, j);
	}

	// left face
	for (i = -2; i <= 2; i++)
	{
		glVertex3f(-2, i, -2);
		glVertex3f(-2, i, 2);
	}
	for (j = -2; j <= 2; j++)
	{
		glVertex3f(-2, -2, j);
		glVertex3f(-2, 2, j);
	}

	// right face
	for (i = -2; i <= 2; i++)
	{
		glVertex3f(2, i, -2);
		glVertex3f(2, i, 2);
	}
	for (j = -2; j <= 2; j++)
	{
		glVertex3f(2, -2, j);
		glVertex3f(2, 2, j);
	}

	glEnd();
	glPopMatrix();
}// End DrawBoundingBox(..)

