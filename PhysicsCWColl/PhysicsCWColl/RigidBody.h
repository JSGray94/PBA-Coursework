/*
This header file has been created to help with the understanding of
the rigidbody.h file.
*/

#ifndef TESTRIGIDBODY_H
#define TESTRIGIDBODY_H

//Include other header files that help with:
//Maths
//Graphics
#include "MathsHelp.h"
#include "GraphicsHelp.h"

//This is the main rigid body class.
//This holds information about rigid body objects.
class TestRigidBody
{
public:
	//Basic rigid body variables.
	/*
	LINEAR
	*/
	Vector3 mPosition;
	float mInvMass;
	Vector3 mLinearVelocity;
	Vector3 mForces;
	/*
	Angular
	*/
	Quaternion mOrientation;
	Matrix4 mLocalInvInertia;
	Vector3 mAngleVelocity;
	Vector3 mTorques;

	//Custom optimisation parameters.
	Matrix4		mMatWorld;
	Matrix4		mWorldInvInertia;
	float		mRadius;

	//Cube variables.
	Vector3		mLocalxyz[3];	// Local x-y-z Axes
	Vector3		mHalfExtends;	// Positive halfwidths along each axis

	//-----------------------------------------------------------------------------------------------------------
	//-----------------------------------------------------------------------------------------------------------

	//Rigid Body constructor.
	//Takes in POSITION(X,Y,Z), ROTATION(X,Y,Z), DIMENTIONS(X, Y, Z) and INVERSEMASS float.
	TestRigidBody(const Vector3& pos, const Vector3& rot, const Vector3& dims, float invMass)
	{
		//Set parameter inputs to object attributes.
		mPosition = pos;
		mInvMass = invMass;
		mLinearVelocity = Vector3(0, 0, 0);
		mForces = Vector3(0, 0, 0);

		Quaternion QX = Quaternion::FromAxisAngle(Vector3(1, 0, 0), rot.x);	//Rotation on the X axis
		Quaternion QY = Quaternion::FromAxisAngle(Vector3(0, 1, 0), rot.y);	//Rotation on the Y axis.
		Quaternion QZ = Quaternion::FromAxisAngle(Vector3(0, 0, 1), rot.z);	//Rotation on the Z axis.
		mOrientation = QX * QY * QZ;										//The sum of all of the rotation stored.
		mOrientation = Quaternion::Normalize(mOrientation);					//Must normalize to apply to vector.

		//Set angle velocity and torque initial values to 0.
		mAngleVelocity = Vector3(0, 0, 0);
		mTorques = Vector3(0, 0, 0);

		// Local inertia calculation for a cube
		Vector3 size = dims * 2.0f;

		float x2 = (size.x * size.x);
		float y2 = (size.y * size.y);
		float z2 = (size.z * size.z);
		float ix = (y2 + z2) / (invMass * 12.0f);
		float iy = (x2 + z2) / (invMass * 12.0f);
		float iz = (x2 + y2) / (invMass * 12.0f);
		// Inverse inertia matrix
		ix = 1.0f / ix;
		iy = 1.0f / iy;
		iz = 1.0f / iz;

		//a 4x4 matrix 
		mLocalInvInertia = Matrix4(ix, 0, 0, 0,
			0, iy, 0, 0,
			0, 0, iz, 0,
			0, 0, 0, 1);

		mRadius = sqrtf(dims.x*dims.x + dims.y*dims.y + dims.z*dims.z) + 0.1f;	//Floating point radius from SQUARE ROOT OF: [(X*X + Y*Y + Z*Z) + 0.1].

		mHalfExtends = dims * 0.5;

		// Update mWorldInvInertia and mWorld 
		updateMatrix();

	}//END CONSTRUCTOR TestRigidBody

	//Update Matrix FUNCTION.
	void updateMatrix() {

		Matrix4 matR = Quaternion::ToRotationMatrix(mOrientation);	//Rotation Matrix
		Matrix4 matT = Matrix4::SetTranslation(mPosition);			//Translation matrix
		mMatWorld = matR * matT;									//World matrix = matR * matT.
		mWorldInvInertia = Matrix4::Transpose(matR) * mLocalInvInertia * matR;	//Set WorldInversInertia: TransposeMatrix * LocalInverseMatrix * RotationMatrix.

		mLocalxyz[0] = Matrix4::Transform(matR, Vector3(1, 0, 0));	//Set cube x axis to vec3(1, 0, 0)
		mLocalxyz[1] = Matrix4::Transform(matR, Vector3(0, 1, 0));	//Set cube y axis to vec3(0, 1, 0)
		mLocalxyz[2] = Matrix4::Transform(matR, Vector3(0, 0, 1));	//Set cube z axis to vec3(0. 0. 1)

	}//END FUNCTION

	//This renders the rigid body to the screen.
	void TestRigidBody::Render()
	{
		//Draw cube, takes in (Mat4, Quaternion, Float, int, int, int)
		DrawCube(mPosition, mOrientation, mHalfExtends*2.0f, 1, 0, 0);
		//Update matrix with new values.
		updateMatrix();

	}//END Render FUNCTION.

	//Integrate the rigid body. Takes in float representing time.
	void TestRigidBody::Integrate(float dt)
	{
		//Update the current matrix.
		updateMatrix();

		//Add gravity.
		//Call add force FUNCTION and pass in translation/position, vec3(X, Y, Z).
		//Apply gravity to the Y axis (F = ma)
		addForce(mPosition, Vector3(0, -9.8f * (1.0f / mInvMass), 0));

		// Update Linear
		mLinearVelocity += ((mForces * mInvMass) * dt);	//Linear Velocity = (vec3 Forces * float Mass) * time
		mPosition += (mLinearVelocity * dt);			//vec3 position(X, Y, Z) = Linear Velocity * time
		mForces = Vector3(0, 0, 0);						//Set vec3 Forces to (0, 0, 0)

		// Update Angular
		mAngleVelocity += (Matrix4::Transform(mWorldInvInertia, mTorques) * dt);	//Vec3 angular velocity = mat4 Transform by world inertia and torque * time
		Quaternion Qvel = (Quaternion(mAngleVelocity.x, mAngleVelocity.y, mAngleVelocity.z, 0) * 0.5f) * mOrientation;	//Rotation velocity on all three axises.
		mOrientation += Qvel * dt;	//Set orientation to rotation velocity * time.

		mOrientation = Quaternion::Normalize(mOrientation);	//Normalize quaternion to apply to vec3
		mTorques = Vector3(0, 0, 0);	//Set initial torque to 0, 0, 0

		//Update the matrix after these changes have been made.
		updateMatrix();

	}//END Integrate FUNCTION.

	//A FUNCTION that takes in position and force vectors then applies force.
	void addForce(Vector3 &p, Vector3 &f)
	{
		//If the force is neglegible then just return out of the function.
		if (mInvMass <= 0.0f)
			return;

		//Increment Force (f = f+f)
		//Increment torque
		mForces += f;
		mTorques += Vector3::Cross((p - mPosition), f);

	}//END addForce FUNCTION

	//This FUNCTION takes in:
	//1: 2 rigid body objects
	//2: vec3 hitpoint - point of contact between 2 objects
	//3: vec3 normal - the normal of the point of contact
	//4: float penetration - level of penetration between rigid bodies
	//This FUNCTION will calculate the impulse produced from 2 rigid bodies colliding
	static
		void AddCollisionImpulse(TestRigidBody&		c0,
		TestRigidBody&		c1,
		const Vector3&	hitPoint,
		const Vector3&	normal,
		float			penetration)
	{
		//SETS
		//Create new floats for invMass and set to both rigid bodies InvMasses.
		const float invMass0 = c0.mInvMass;
		const float invMass1 = c1.mInvMass;
		//Create new 4x4 matrices to hold each objects worldInvInertia.
		const Matrix4& worldInvInertia0 = c0.mWorldInvInertia;
		const Matrix4& worldInvInertia1 = c1.mWorldInvInertia;

		// Both objects are non movable exit out of function.
		if ((invMass0 + invMass1) == 0.0)
			return;

		//New vectors represent contact points on rigid bodies
		//Take vec3 hitpoint and subtract position vectors.
		Vector3 r0 = hitPoint - c0.mPosition;
		Vector3 r1 = hitPoint - c1.mPosition;
		//Sum up angular and linear forces.
		Vector3 v0 = c0.mLinearVelocity + Vector3::Cross(c0.mAngleVelocity, r0);
		Vector3 v1 = c1.mLinearVelocity + Vector3::Cross(c1.mAngleVelocity, r1);

		// Relative Velocity
		Vector3 dv = v0 - v1;

		// If the objects are moving away from each other we dont need to apply an impulse
		float relativeMovement = -Vector3::Dot(dv, normal);
		if (relativeMovement < -0.01f)
		{
			return;
		}

		// NORMAL Impulse code
		{
			// Coefficient of Restitution
			float e = 0.0f;

			float normDiv =	 //Vector3::Dot(normal, normal) * => should equal 1
				((invMass0 + invMass1) +
				Vector3::Dot(normal,
				Vector3::Cross(Matrix4::Transform(worldInvInertia0, Vector3::Cross(r0, normal)), r0) +
				Vector3::Cross(Matrix4::Transform(worldInvInertia1, Vector3::Cross(r1, normal)), r1)));

			float jn = -1 * (1 + e)*Vector3::Dot(dv, normal) / normDiv;

			// Hack fix to stop sinking - bias impulse proportional to penetration distance
			jn = jn + (penetration*1.5f);

			c0.mLinearVelocity += normal * jn * invMass0;
			c0.mAngleVelocity += Matrix4::Transform(worldInvInertia0, Vector3::Cross(r0, normal * jn));

			c1.mLinearVelocity -= normal * jn * invMass1;
			c1.mAngleVelocity -= Matrix4::Transform(worldInvInertia1, Vector3::Cross(r1, normal * jn));
		}


		// TANGENT Impulse Code
#if 1
		{
			// Work out our tangent vector, with is perpendicular
			// to our collision normal
			Vector3 tangent = Vector3(0, 0, 0);
			tangent = dv - (normal * Vector3::Dot(dv, normal));
			if (tangent.LengthSq() > 0.0f)
				tangent = Vector3::Normalize(tangent);

			float tangDiv = invMass0 + invMass1 +

				Vector3::Dot(tangent,
				Vector3::Cross(Matrix4::Transform(worldInvInertia0, Vector3::Cross(r0, tangent)), r0) +
				Vector3::Cross(Matrix4::Transform(worldInvInertia1, Vector3::Cross(r1, tangent)), r1)
				);

			float jt = -1 * Vector3::Dot(dv, tangent) / tangDiv;
			// Clamp min/max tangental component

			// Apply contact impulse
			c0.mLinearVelocity += tangent * jt * invMass0;
			c0.mAngleVelocity += Matrix4::Transform(worldInvInertia0, Vector3::Cross(r0, tangent * jt));

			c1.mLinearVelocity -= tangent * jt * invMass1;
			c1.mAngleVelocity -= Matrix4::Transform(worldInvInertia1, Vector3::Cross(r1, tangent * jt));
		}
#endif
		// TANGENT

	}//END addCilloisionImpulse FUNCTION

};//END CLASS TestRigidBody


#endif	//RIGIDBODY_H