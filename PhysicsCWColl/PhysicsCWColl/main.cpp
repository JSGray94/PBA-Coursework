/*
// The main entry point of the simulation.
// Holds information about 3D vectors.
// Author:
// Jordan S. Gray
// 40087220
// Edinburgh Napier University
// BSc Games Development
// Year 3
// graybostephano@gmail.com
*/

//Declare any header files and libraries needed.
//Basic standard libs.
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <set>
#include <stdlib.h>
#include <vector>
#include <sstream>
#include <ctime>

//Include the glut header file.
#include <GL\glut.h>

//Include local header files.
#include "ImageLoader.h"
#include "Vec3f.h"
#include "GraphicsHelp.h"
#include "MathsHelp.h"
#include "RigidBody.h"
#include "TestCubeCollision.h"

//include any namespaces.
using namespace std;

//--------------------------------------------------
//--------------------------------------------------

//GLOBALS!

//Default gravity
const float GRAVITY = 9.82f;
const float DAMPING = 0.90f;
const float BOX_SIZE = 12.0f; //The length of one side of the box
//The amount of time between each time that we handle collisions and apply the effects of gravity
const float TIME_BETWEEN_UPDATES = 0.01f; //Time between updates determines how often updates run.
const int TIMER_MS = 25; //The number of milliseconds to which the timer is set

const int MAX_OCTREE_DEPTH = 6;	//Max depth of the octree
const int MIN_BALLS_PER_OCTREE = 3;	//Minmun balls per octree
const int MAX_BALLS_PER_OCTREE = 6;	//Max balls per octree

static bool slowMethod = false;		//Required for switching between broad phase to just narrow phase

static int ballCount = 0;	//Count number of balls in scene.

//GLOBALS!


//Returns a random float from 0 to < 1
float randomFloat()
{
	return (float)rand() / ((float)RAND_MAX + 1);

}

//--------------------------------------------------
//--------------------------------------------------

//Stores ball/sphere information
struct Ball {
	Vec3f velocity; //Velocity
	Vec3f pos; //Position
	float radius; //Radius
	Vec3f colour; //Colour
};

//Wall enumerations
//Ordered listing of items in a collection
enum Wall 
{
	WALL_LEFT, WALL_RIGHT, WALL_FAR, WALL_NEAR, WALL_TOP, WALL_BOTTOM 
};

//Stores two balls/spheres
struct BallPair 
{
	Ball* ball1;
	Ball* ball2;
};

//Stores a ball and a wall pair
struct BallWallPair 
{
	Ball* ball;
	Wall wall;
};

//--------------------------------------------------
//--------------------------------------------------

/*
// An octree data structure.
// This data structure is used to work out potential collisions between
// Objects (balls in this case) in a scene. 
// Octrees are made up of nodes (sections in the scene) which each have 8 children
// up to a set accuracy.
*/
class Octree 
{
private:
	Vec3f corner1;	//(minX, minY, minZ) vector
	Vec3f corner2;	//(maxX, maxY, maxZ) vector
	Vec3f center;	//((minX + maxX) / 2, (minY + maxY) / 2, (minZ + maxZ) / 2) vector

	/*
	// If has any children.
	// Children[0][*][*] = children with x co-ords ranging from minX to centreX.
	// Children[1][*][*] = children with x co-ords ranging from centreX to maxX.
	// Same goes for both Y dimensions and Z dimensions.
	*/
	Octree *children[2][2][2];
	//Whether this has children
	bool hasChildren;
	//The balls in this, if this doesn't have any children
	set<Ball*> ballSet;
	//The depth of this in the tree
	int depth;
	//The number of balls in this includes children.
	int numBalls;

	//Adds a ball to or removes one from the children of this
	//Takes in ball object, position (x, y, z) and true or false addBall.
	void fileBall(Ball* ball, Vec3f pos, bool addBall) 
	{
		//Figure out in which child(ren) the ball belongs
		//FOR X
		for (int x = 0; x < 2; x++) 
		{
			if (x == 0) {
				if (pos[0] - ball->radius > center[0]) 
				{
					continue;
				}
			}
			else if (pos[0] + ball->radius < center[0]) 
			{
				continue;
			}
			//FOR Y
			for (int y = 0; y < 2; y++) 
			{
				if (y == 0) {
					if (pos[1] - ball->radius > center[1]) 
					{
						continue;
					}
				}
				else if (pos[1] + ball->radius < center[1]) 
				{
					continue;
				}
				//FOR Z
				for (int z = 0; z < 2; z++)
				{
					if (z == 0) {
						if (pos[2] - ball->radius > center[2])
						{
							continue;
						}
					}
					else if (pos[2] + ball->radius < center[2])
					{
						continue;
					}

					//Add or remove the ball
					//Add if bool addBall = true
					if (addBall)
					{
						children[x][y][z]->add(ball);
					}
					//Remove ball if bool addBall = false
					else 
					{
						children[x][y][z]->remove(ball, pos);
					}
				} //END FOR Z
			} //END FOR Y
		} //END FOR X

	} //END FUNCTION void fileBall(...)

	//This Function creates children of this, and moves the balls in this to the children
	void haveChildren() 
	{
		//FOR X
		for (int x = 0; x < 2; x++) 
		{
			float minX;
			float maxX;
			if (x == 0) 
			{
				minX = corner1[0];
				maxX = center[0];
			}
			else 
			{
				minX = center[0];
				maxX = corner2[0];
			}
			//FOR Y
			for (int y = 0; y < 2; y++) 
			{
				float minY;
				float maxY;
				if (y == 0) 
				{
					minY = corner1[1];
					maxY = center[1];
				}
				else 
				{
					minY = center[1];
					maxY = corner2[1];
				}
				//FOR Z
				for (int z = 0; z < 2; z++) 
				{
					float minZ;
					float maxZ;
					if (z == 0) 
					{
						minZ = corner1[2];
						maxZ = center[2];
					}
					else 
					{
						minZ = center[2];
						maxZ = corner2[2];
					}

					//Create new Octree object from this data
					// Takes in 2 vec3's. One for each corner and an int depth.
					children[x][y][z] = new Octree(Vec3f(minX, minY, minZ), Vec3f(maxX, maxY, maxZ), depth + 1);

				} //END FOR Z
			} //END FOR Y
		} //END FOR Z

		//Remove all balls from "balls" and add them to the new children
		//For every ball in the set DO
		for (set<Ball*>::iterator it = ballSet.begin(); it != ballSet.end(); it++) 
		{
			Ball* ball = *it;
			fileBall(ball, ball->pos, true);
		}
		//Clear balls.
		ballSet.clear();
		//Set hasChildren = true
		hasChildren = true;

	} //END FUNCTION void haveChildren(...)

	//Adds all balls in this or one of its descendants to the specified set
	//Takes in a ball set
	void collectBalls(set<Ball*> &bs) 
	{	//If hasChildren = true
		if (hasChildren) 
		{	//FOR X
			for (int x = 0; x < 2; x++) 
			{	//FOR Y
				for (int y = 0; y < 2; y++) 
				{	//FOR Z
					for (int z = 0; z < 2; z++) 
					{
						children[x][y][z]->collectBalls(bs);
					}
				}
			}
		}
		//If hasChildren = false
		else 
		{	//For every ball in the set DO
			for (set<Ball*>::iterator it = ballSet.begin(); it != ballSet.end(); it++) 
			{
				Ball* ball = *it;
				bs.insert(ball);
			}
		}
	}//END FUNCTION void collectBalls(..)

	//Destroys the children of this, and moves all balls in its descendants to the "balls" set
	void destroyChildren() 
	{
		//Move all balls in descendants of this to the "balls" set
		collectBalls(ballSet);
		//FOR X
		for (int x = 0; x < 2; x++) 
		{	//FOR Y
			for (int y = 0; y < 2; y++) 
			{	//FOR Z
				for (int z = 0; z < 2; z++) 
				{
					delete children[x][y][z];
				}
			}
		}

		hasChildren = false;

	} //END FUNCTION void destroyChildren(...)

	//Removes the specified ball at the indicated position
	//Takes in Ball object and position
	void remove(Ball* ball, Vec3f pos) 
	{
		//Decrement numBalls
		numBalls--;

		//If hasChildren = true AND numer of balls < Min balls allowed per octree
		if (hasChildren && numBalls < MIN_BALLS_PER_OCTREE) 
		{
			destroyChildren();
		}
		//If hasChildren = true
		if (hasChildren) 
		{
			fileBall(ball, pos, false);
		}
		else 
		{
			ballSet.erase(ball);
		}
	}

	/*
	// Adds potential ball-wall collisions to BallWallPair vector cs.
	// wallType is type of wall.
	// coord is coord of the wall ("x", "y", "z")
	// dir is 0 if wall in negative direction and 1 if positive direction
	*/
	void potentialBallWallCollisions(vector<BallWallPair> &cs, Wall wallType, char coord, int dir) 
	{
		if (hasChildren) 
		{
			//Recursively call potentialBallWallCollisions on the correct
			//half of the children (if w is WALL_TOP, call it on
			//children above centerY)
			for (int dir2 = 0; dir2 < 2; dir2++) 
			{
				for (int dir3 = 0; dir3 < 2; dir3++) 
				{
					Octree *child = nullptr;
					switch (coord) 
					{
					case 'x':
						child = children[dir][dir2][dir3];
						break;
					case 'y':
						child = children[dir2][dir][dir3];
						break;
					case 'z':
						child = children[dir2][dir3][dir];
						break;
					}
					//Work out potential wall collisions for the child
					//Recursive call												**ERROR? DEBUG HERE**
					child->potentialBallWallCollisions(cs, wallType, coord, dir);
				}
			}
		}
		else 
		{
			//Add (ball, wallType) for all balls in this
			for (set<Ball*>::iterator it = ballSet.begin(); it != ballSet.end(); it++) 
			{
				Ball* ball = *it;
				BallWallPair bwp;
				bwp.ball = ball;
				bwp.wall = wallType;
				cs.push_back(bwp);	//Add here!
			}
		}

	} //END FUNCTION potentialBallWallCollisions(...)

public:
	//Constructs a new Octree.  
	//c1 is (minX, minY, minZ) 
	//c2 is (maxX, maxY,maxZ)
	//d is the depth, which starts at 1.
	Octree(Vec3f c1, Vec3f c2, int d) 
	{
		corner1 = c1;
		corner2 = c2;
		center = (c1 + c2) / 2;
		depth = d;
		numBalls = 0;
		hasChildren = false;
	} 

	//Destructor
	//Remove all children
	~Octree() 
	{
		if (hasChildren) 
		{
			destroyChildren();
			cout << "Destroy Children" << endl;
		}
	}

	//Adds a ball to this
	void add(Ball* ball) 
	{
		numBalls++;	//Increment number of balls

		//If hasChildren = false AND depth is < max octree depth
		if (!hasChildren && depth < MAX_OCTREE_DEPTH && numBalls > MAX_BALLS_PER_OCTREE) 
		{
			haveChildren();
		}
		//If hasChildren = true
		if (hasChildren) 
		{
			fileBall(ball, ball->pos, true);
		}
		else 
		{
			ballSet.insert(ball);
		}

	} //END FUNCTION void add(...)

	//Removes a ball from this
	void remove(Ball* ball) 
	{
		remove(ball, ball->pos);
	} //END FUNCTION void remove(...)

	//Changes the position of a ball in this from oldPos to ball->pos
	void ballMoved(Ball* ball, Vec3f oldPos) 
	{
		remove(ball, oldPos);
		add(ball);

	} //END FUNCTION void ballMoved(...)

	//Adds potential ball-ball collisions to the specified set (PAIRS)
	//Takes in a vector of ball pairs and tests for potential collisions
	void potentialBallBallCollisions(vector<BallPair> &collisions) 
	{	//If hasChildren = true
		if (hasChildren) 
		{	//FOR X
			for (int x = 0; x < 2; x++) 
			{	//FOR Y
				for (int y = 0; y < 2; y++) 
				{	//FOR Z
					for (int z = 0; z < 2; z++) 
					{
						children[x][y][z]->potentialBallBallCollisions(collisions);	//Tests for children.
					}
				}
			}
		}
		else 
		{
			//Add all pairs (ball1, ball2) from balls
			for (set<Ball*>::iterator it = ballSet.begin(); it != ballSet.end(); it++) 
			{
				Ball* ball1 = *it;
				//For every ball in the set DO
				for (set<Ball*>::iterator it2 = ballSet.begin(); it2 != ballSet.end(); it2++) 
				{
					Ball* ball2 = *it2;
					//This test makes sure that I only add each pair once
					if (ball1 < ball2) 
					{
						BallPair bp;
						bp.ball1 = ball1;
						bp.ball2 = ball2;
						collisions.push_back(bp);
					}
				}
			} //END FOR
		} //END ELSE

	} //END FUNCTION potentialBallBallCollisions(...)

	//Adds potential ball-wall collisions to the specified set
	void potentialBallWallCollisions(vector<BallWallPair> &collisions) 
	{
		potentialBallWallCollisions(collisions, WALL_LEFT, 'x', 0);
		potentialBallWallCollisions(collisions, WALL_RIGHT, 'x', 1);
		potentialBallWallCollisions(collisions, WALL_BOTTOM, 'y', 0);
		potentialBallWallCollisions(collisions, WALL_TOP, 'y', 1);
		potentialBallWallCollisions(collisions, WALL_FAR, 'z', 0);
		potentialBallWallCollisions(collisions, WALL_NEAR, 'z', 1);
	}
};

//--------------------------------------------------
//--------------------------------------------------

//Puts potential ball-ball collisions in potentialCollisions.  
//It must return all actual collisions.
//Doesn need to return just collisions
void potentialBallBallCollisions(vector<BallPair> &potentialCollisions, vector<Ball*> &balls, Octree* octree) 
{
	if (slowMethod == false)
	{
		//Fast method calls on the OCTREE
		octree->potentialBallBallCollisions(potentialCollisions);
	}
	else
	{
		
		//Slow method
		for(unsigned int i = 0; i < balls.size(); i++)
		{
			for(unsigned int j = i + 1; j < balls.size(); j++)
			{
				BallPair bp;
				bp.ball1 = balls[i];
				bp.ball2 = balls[j];
				potentialCollisions.push_back(bp);
			}
		}
		
	}

} //END FUNCTION potentialBallBallCollisions

//Puts potential ball-wall collisions in potentialCollisions.  
//It must return all actual collisions, but doesnt need to return only actual collisions.
void potentialBallWallCollisions(vector<BallWallPair> &potentialCollisions, vector<Ball*> &balls, Octree* octree) 
{
	if (slowMethod == false)
	{
		//Fast method acts on OCTREE
		octree->potentialBallWallCollisions(potentialCollisions);
	}
	else
	{
		//Slow method
		Wall walls[] = { WALL_LEFT, WALL_RIGHT, WALL_FAR, WALL_NEAR, WALL_TOP, WALL_BOTTOM };
		for (unsigned int i = 0; i < balls.size(); i++)
		{
			for (int j = 0; j < 6; j++)
			{
				BallWallPair bwp;
				bwp.ball = balls[i];
				bwp.wall = walls[j];
				potentialCollisions.push_back(bwp);
			}
		}
	}

} //END FUNCTION potentialBaallWallCollisions(...)

//Moves all of the balls by their velocity times dt
//Takes in vector of balls, Octree object and float delta time.
void moveBalls(vector<Ball*> &balls, Octree* octree, float dt) 
{
	//For every ball in the passed in ball set DO
	for (unsigned int i = 0; i < balls.size(); i++) 
	{
		Ball* ball = balls[i];	//ball = current indexed ball in set
		Vec3f oldPos = ball->pos;	//Update position
		ball->pos += ball->velocity * dt;	//Equate in forces
		octree->ballMoved(ball, oldPos);
	}

} //END FUNCTION void moveBalls(...)

//Decreases the y coordinate of the velocity of each ball by GRAVITY * TIME_BETWEEN_UPDATES
void applyGravity(vector<Ball*> &balls) 
{	//For every ball in passed in ball set DO
	for (unsigned int i = 0; i < balls.size(); i++) 
	{
		Ball* ball = balls[i];	//Set ball to current indexed ball in set
		ball->velocity -= Vec3f(0, GRAVITY * TIME_BETWEEN_UPDATES, 0); //Apply gravity to velocity
	}

} //END FUNCTION applyGravity(...)

//Apply damping to the velocity of the balls for gradual slow down.
void applyDamping(vector<Ball*> &balls)
{
	//For every ball passed in DO
	for (unsigned int i = 0; i < balls.size(); i++)
	{
		Ball* ball = balls[i];	//Set ball obj to current indexed ball
		ball->velocity -= Vec3f(DAMPING * TIME_BETWEEN_UPDATES, DAMPING * TIME_BETWEEN_UPDATES, DAMPING * TIME_BETWEEN_UPDATES);
	}
}

//Returns whether two balls are colliding
//Takes in 2 ball objects
bool testBallBallCollision(Ball* b1, Ball* b2) 
{
	//Check whether the balls are close enough
	//BASIC SPHERE SPHERE NARROW PHASE
	float r = b1->radius + b2->radius;
	//If the distance between both spheres radius causes collision
	if ((b1->pos - b2->pos).magnitudeSquared() < r * r) 
	{
		//Check whether the balls are moving toward each other
		Vec3f netVelocity = b1->velocity - b2->velocity;
		Vec3f displacement = b1->pos - b2->pos;
		return netVelocity.dot(displacement) < 0;
	}
	else
		return false;

} //END FUNCTION testBallBallCollision(...)

//Handles all ball-ball collisions
//Takes in vector of Ball objects and an Octree object
void handleBallBallCollisions(vector<Ball*> &balls, Octree* octree) 
{	//Declare ball pair
	vector<BallPair> bps;
	//Call potential ball ball collision function
	potentialBallBallCollisions(bps, balls, octree);
	//For every ball in the ball pair (both) DO
	for (unsigned int i = 0; i < bps.size(); i++) 
	{
		BallPair bp = bps[i];

		Ball* b1 = bp.ball1;
		Ball* b2 = bp.ball2;

		//If there is a collision between the two balls
		if (testBallBallCollision(b1, b2)) 
		{
			//Make the balls reflect off of each other
			Vec3f displacement = (b1->pos - b2->pos).normalize();
			b1->velocity -= 2 * displacement * b1->velocity.dot(displacement) * DAMPING;
			b2->velocity -= 2 * displacement * b2->velocity.dot(displacement) * DAMPING;
		}
	}

} //END FUNCTION void handleBallBallCollisions(...)

//Returns the direction from the origin to the wall
//Takes in Wall object and returns position (vec3)
Vec3f wallDirection(Wall wall) 
{
	switch (wall) 
	{
	case WALL_LEFT:
		return Vec3f(-1, 0, 0);
	case WALL_RIGHT:
		return Vec3f(1, 0, 0);
	case WALL_FAR:
		return Vec3f(0, 0, -1);
	case WALL_NEAR:
		return Vec3f(0, 0, 1);
	case WALL_TOP:
		return Vec3f(0, 1, 0);
	case WALL_BOTTOM:
		return Vec3f(0, -1, 0);
	default:
		return Vec3f(0, 0, 0);
	}
} //END FUNCTION Vec3f wallDirection(...)

//Returns whether a ball and a wall are colliding
//Takes in a ball object and a wall object and returns boolean based on collision.
bool testBallWallCollision(Ball* ball, Wall wall) 
{
	Vec3f dir = wallDirection(wall);
	//Check whether the ball is far enough in the "dir" direction, and whether
	//it is moving toward the wall
	return ball->pos.dot(dir) + ball->radius > BOX_SIZE / 2 && ball->velocity.dot(dir) > 0;

} //END FUNCTION bool testBallWallCollision(...)

//Handles all ball-wall collisions
//Takes in a vector of Ball objects and an octree object.
void handleBallWallCollisions(vector<Ball*> &balls, Octree* octree) 
{
	vector<BallWallPair> bwps;
	potentialBallWallCollisions(bwps, balls, octree);

	//For every ball in the vector of balls passed in 
	for (unsigned int i = 0; i < bwps.size(); i++) 
	{
		BallWallPair bwp = bwps[i];

		Ball* b = bwp.ball;
		Wall w = bwp.wall;
		if (testBallWallCollision(b, w)) 
		{
			//Make the ball reflect off of the wall
			Vec3f dir = (wallDirection(w)).normalize();
			b->velocity -= 2 * dir * b->velocity.dot(dir) * DAMPING;
		}
	}

} //END FUNCTION handleBallWallCollisions(...)

//Applies gravity and handles all collisions.  Should be called every
//TIME_BETWEEN_UPDATES seconds.											**APPLY DAMPENING HERE**
void performUpdate(vector<Ball*> &balls, Octree* octree) 
{
	applyGravity(balls);	//Apply grvity
	applyDamping(balls);	//Apply damping
	handleBallBallCollisions(balls, octree);	//Handle ball ball collisions
	handleBallWallCollisions(balls, octree);	//Handle ball wall collisions

} //END FUNCTION void performUpdate(...)

//Advances the state of the balls by t 
//timeUntilUpdate is the amount of time until the next call to performUpdate.
//Takes in vector of Ball objects, Octree object, time and tme til next update
void tick(vector<Ball*> &balls, Octree* octree, float t, float &timeUntilUpdate) 
{
	//Perform the updates/ticks
	while (t > 0) 
	{
		if (timeUntilUpdate <= t) 
		{
			moveBalls(balls, octree, timeUntilUpdate);
			performUpdate(balls, octree);
			t -= timeUntilUpdate;
			timeUntilUpdate = TIME_BETWEEN_UPDATES;
		}
		else 
		{
			moveBalls(balls, octree, t);
			timeUntilUpdate -= t;
			t = 0;
		}

	} //END WHILE

} //END FUNCTION void tick(...)


//--------------------------------------------------
//--------------------------------------------------

//All of the balls in play
vector<Ball*> _balls; 
//The camera angle
float _angle = 0.0f; 
//An octree with all af the balls
Octree* _octree; 
//The amount of time until performUpdate should be called
float _timeUntilUpdate = 0;
GLuint _textureId;

//Deletes everything called when exiting program.
void cleanup() 
{	//For every ball in the set detelet each ball.
	for (unsigned int i = 0; i < _balls.size(); i++) 
	{
		delete _balls[i];
	}
	//Deleted octree object.
	delete _octree;
}

//This function deals with key presses
void handleKeypress(unsigned char key, int x, int y) 
{
	switch (key) 
	{
	case 27: //Escape key
		cleanup();
		exit(0);
	case ' ':	//Space key
		//Add 20 balls with a random position, velocity, radius, and color
		for (int i = 0; i < 20; i++) 
		{
			Ball* ball = new Ball();
			//Generate random position
			ball->pos = Vec3f(8 * randomFloat() - 4, 8 * randomFloat() - 4, 8 * randomFloat() - 4);
			//Generate random velocity
			ball->velocity = Vec3f(8 * randomFloat() - 4, 8 * randomFloat() - 4, 8 * randomFloat() - 4);
			//Generate random radius
			ball->radius = 0.1f * randomFloat() + 0.1f;
			//Generate random colour
			ball->colour = Vec3f(0.6f * randomFloat() + 0.2f, 0.6f * randomFloat() + 0.2f, 0.6f * randomFloat() + 0.2f);
			//Add to ball set and to octree
			_balls.push_back(ball);
			_octree->add(ball);

		}
		//Add to ball count
		ballCount += 20;
		break;
	case VK_RETURN:	//Return Key

		slowMethod = !slowMethod;	//Switches broadphase technique off and on.

		break;
		
	}

} //END FUNCTION void handleKeyPress(...)

//Draw the number of balls to the screen.
void displayBallCount(string caption, int ballCount, float r, float g, float b, float x, float y, float z)
{
	//Initialise colour
	glColor3f(r, g, b);
	//Initialise position
	glRasterPos3f(x, y, z);

	//Set up string stream
	stringstream strm;
	strm << caption << ballCount;
	string text = strm.str();
	for (string::iterator it = text.begin(); it != text.end(); ++it) 
	{
		glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, *it);
	}

} //END FUNCTION displayBallCount(...)

//Initialise rendering.
void initRendering() 
{
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_NORMALIZE);
	glEnable(GL_COLOR_MATERIAL);

} //END FUNCTION void initRendering(...)

//Takes in int width and int height
void handleResize(int w, int h) 
{
	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0, (float)w / (float)h, 1.0, 200.0);

} //END FUNCTION void handleResize(...)

//Calculate FPS in scene.
void calcFPS()
{
	
}

//This handles the final drawing of the scene to be called in the main method.
void drawScene() 
{
	//CLEAR
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//Dislay ball count here
	displayBallCount("Sphere Count: ", ballCount, 0.0f, 1.0f, 1.0f, -1.0f, 1.0f, 1.0f);

	//TRANFORMS
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslatef(0.0f, 0.0f, -20.0f);
	glRotatef(-_angle, 0.0f, 1.0f, 0.0f);

	//Ambient light
	GLfloat ambientColor[] = { 0.5f, 0.5f, 0.5f, 1.0f };
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambientColor);

	GLfloat lightColor[] = { 0.7f, 0.7f, 0.7f, 1.0f };
	GLfloat lightPos[] = { 1.0f, 0.2f, 0.0f, 0.0f };
	glLightfv(GL_LIGHT0, GL_DIFFUSE, lightColor);
	glLightfv(GL_LIGHT0, GL_POSITION, lightPos);

	//Draw the top and the bottom of the box
	glShadeModel(GL_FLAT);
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, _textureId);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glColor3f(1.0f, 1.0f, 1.0f);
	glBegin(GL_QUADS);

	//DRAW BOX ROOM
	glNormal3f(0.0f, 1.0f, 0.0f);
	glTexCoord2f(0.0f, 0.0f);
	glVertex3f(-BOX_SIZE / 2, -BOX_SIZE / 2, -BOX_SIZE / 2);
	glTexCoord2f(1.0f, 0.0f);
	glVertex3f(-BOX_SIZE / 2, -BOX_SIZE / 2, BOX_SIZE / 2);
	glTexCoord2f(1.0f, 1.0f);
	glVertex3f(BOX_SIZE / 2, -BOX_SIZE / 2, BOX_SIZE / 2);
	glTexCoord2f(0.0f, 1.0f);
	glVertex3f(BOX_SIZE / 2, -BOX_SIZE / 2, -BOX_SIZE / 2);

	glNormal3f(0.0f, -1.0f, 0.0f);
	glTexCoord2f(1.0f, 0.0f);
	glVertex3f(-BOX_SIZE / 2, BOX_SIZE / 2, -BOX_SIZE / 2);
	glTexCoord2f(1.0f, 1.0f);
	glVertex3f(BOX_SIZE / 2, BOX_SIZE / 2, -BOX_SIZE / 2);
	glTexCoord2f(0.0f, 1.0f);
	glVertex3f(BOX_SIZE / 2, BOX_SIZE / 2, BOX_SIZE / 2);
	glTexCoord2f(0.0f, 0.0f);
	glVertex3f(-BOX_SIZE / 2, BOX_SIZE / 2, BOX_SIZE / 2);
	//END DRAW BOX/ROOM

	glEnd();
	glShadeModel(GL_SMOOTH);
	glDisable(GL_TEXTURE_2D);

	//Draw the balls
	for (unsigned int i = 0; i < _balls.size(); i++) 
	{
		Ball* ball = _balls[i];
		glPushMatrix();
		glTranslatef(ball->pos[0], ball->pos[1], ball->pos[2]);
		glColor3f(ball->colour[0], ball->colour[1], ball->colour[2]);
		glutSolidSphere(ball->radius, 12, 12); //Draw a sphere
		glPopMatrix();
	}

	glutSwapBuffers();

} //END FUNCTION void drawScene(...)

//Called every TIMER_MS milliseconds
//Update the variables in the seen every tick.
void update(int value) 
{

	tick(_balls, _octree, (float)TIMER_MS / 1000.0f, _timeUntilUpdate);
	_angle += (float)TIMER_MS / 100;
	if (_angle > 360) 
	{
		_angle -= 360;
	}

	if (slowMethod == false)
	{
		cout << "Broadphase Detection: On " << endl;
	}
	else
	{
		cout << "Broadphase Detection: Off " << endl;
	}

	glutPostRedisplay();
	glutTimerFunc(TIMER_MS, update, 0);
}

//--------------------------------------------------
//--------------------------------------------------

//The main function of the whole program.
int main(int argc, char** argv) 
{
	srand((unsigned int)time(0)); //Seed the random number generator

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(1280, 678);

	glutCreateWindow("Physics Based Animation Simulation");
	initRendering();

	_octree = new Octree(Vec3f(-BOX_SIZE / 2, -BOX_SIZE / 2, -BOX_SIZE / 2),
		Vec3f(BOX_SIZE / 2, BOX_SIZE / 2, BOX_SIZE / 2), 1);

	
	glutDisplayFunc(drawScene);
	glutKeyboardFunc(handleKeypress);
	glutReshapeFunc(handleResize);
	glutTimerFunc(TIMER_MS, update, 0);

	glutMainLoop();
	return 0;
} //END FUNCTION main(...)