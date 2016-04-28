/*
// Working from the cubecollision.h file to build an understanding of the project.
// Basically, you pass in a cubes, and the code will determine the
// collision points, penetration depth and the direction of the
// collision to push the cubes appart.

// Basic collision detection methods
*/

//Declare externals.
#ifndef TESTCUBECOLLISION_H
#define TESTCUBECOLLISION_H

#include "RigidBody.h"

// Each contact point is represented by a point, normal and a penetration depth
//This defines a contact point structure.
struct stContactPoint
{
	Vector3 point;
	Vector3 normal;
	float	penetration;
};

//This FUNCTION returns a boolean (true/false) for object to floor collision.
//It takes in: Rigid Body Object, Vector(array) of contact points
bool cubeFloorCollisionCheck(const TestRigidBody& box0, std::vector<stContactPoint>&	contactPoints) {

	//The cubes attributes
	float x = box0.mHalfExtends.x;
	float y = box0.mHalfExtends.y;
	float z = box0.mHalfExtends.z;

	// 8 corners of a 3D cube
	Vector3 Vertex[8] =
	{
		Vector3(x, y, z),
		Vector3(-x, y, z),
		Vector3(x, -y, z),
		Vector3(-x, -y, z),

		Vector3(x, y, -z),
		Vector3(-x, y, -z),
		Vector3(x, -y, -z),
		Vector3(-x, -y, -z)
	};

	//For each corner of the cube transform box world space 
	for (int i = 0; i<8; i++)
	{
		Vertex[i] = Matrix4::Transform(box0.mMatWorld, Vertex[i]);

#if 1
		::DrawSphere(Vertex[i], 0.2f, 0, 1, 0);
#endif
	}


	//Clear the contact points
	contactPoints.clear();
	//For each corner of the cube DO
	for (int i = 0; i<8; i++)
	{
		// Uncomplicated plane-point check - if the point goes
		// below the plane at (0,0,0) - we detect it and the penetration
		// depth. d = dot product between vertex i and floor plane.
		float d = Vector3::Dot(Vertex[i], Vector3(0, 1, 0));
		//If the vertex point goes below the plane DO
		if (d < 0.0f)
		{
			stContactPoint contactPoint;	//Declare contact point.
			contactPoint.normal = -Vector3(0, 1, 0);	//Find contact point normal
			contactPoint.penetration = -d;	//Contact point penetration = -Force. (Create impulse).
			contactPoint.point = Vertex[i];	//Contact point = what vertex on cube made contact.
			contactPoints.push_back(contactPoint);	//Sum

			//Draw arrows representing the impulse force being generated on cube verteces.
#if 1
			DrawArrow(contactPoint.point,
				contactPoint.point - (contactPoint.normal*(contactPoint.penetration + 0.11f)) * 10.0f, 0.1f);
#endif
		}
	}

	// Check if we have had a collision
	if (contactPoints.size() > 0)
	{
		return true;
	}

	// No Collision
	return false;

}//END cubeFloorCollisionCheck FUNCTION.






#endif //CUBECOLLISION_H