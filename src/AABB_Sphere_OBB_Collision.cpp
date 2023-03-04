#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <ctime>
#include <assert.h>
#include <algorithm>
#include <iostream>
#include <vector>
#include <memory>

#ifdef WIN32
#include <windows.h>
#endif

#ifndef MACOSX
#include <GL/gl.h>
#include <GL/glu.h>
#else
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#endif

#include <math.h>
#include "wcode/fssimplewindow.h"
#include "wcode/fswin32keymap.h"
#include "bitmapfont\ysglfontdata.h"

#include "vectors.h"
#include "Matrices.h"

#include "eig3.h"

using namespace std;
namespace {
	typedef enum
	{
		eIdle = -2,
		eStop = -1,
		eStart = 0,
	} changeType;

	constexpr float PI{ 3.1415926f };
	float iSpeed{ 70.0 };
	float radius{ 40. };
	size_t num_segments{ 20 };
	typedef enum class eCollisionMode
	{
		eSphere,
		eAABB,
		eOBB
	};
	eCollisionMode CollisionType = eCollisionMode::eSphere;

	const struct {
		vector<pair<float, float>> vertPolarCoordinate;
		float3D pos, vel;
		float rotation;
		float rotationVelocity;
	} defpolys[] = { { { pair<float, float>( 1, 100),
					     pair<float, float>( 2, 80 ),
					     pair<float, float>( 4, 90 ),
					     pair<float, float>( 5, 80 ),
					     pair<float, float>( 6, 80 ) }, float3D(195.212677, 151.169327), float3D(-80.43, -67.5), -0.308457941, -1 },
					 { { pair<float, float>( 1, 100),
					     pair<float, float>( 4, 90 ),
					     pair<float, float>( 6, 80 ) }, float3D(262.598, 108.94), float3D(-80.43, 67.5), 0.215983137, .7 } };
	constexpr size_t PolyCount(sizeof defpolys / sizeof *defpolys);
	const unsigned frameRate = 30; // 30 frame per second is the frame rate.
	const int timeSpan = 1000 / frameRate; // milliseconds
	const double timeInc = (double)timeSpan * 0.001; // time increment in seconds

	size_t WinWidth{ 800 }, WinHeight{ 600 };

	///////////////////// physics stuff ///////////////////
	float3D contactPoint;
	float3D normal;
	bool pointSet = false;
	bool normalSet = false;

	bool pause = true;

	struct AABB
	{// pos is center of AABB, extent is half length extend in each direction
		float3D pos{}, extend{};
		float3D LL() const { return pos - extend; }
		float3D UR() const { return pos + extend; }
	};

	struct OBB
	{
		float3D center{}, extend{ -1.f, -1.f, -1.f };
		Matrix3 orientation{};  // shows orientation of OBB in space

		void set(float3D c, float3D e, Matrix3 o)
		{
			center = c;
			extend = e;
			orientation = o;
		}
	};

	struct Sphere
	{
		float3D center{};
		float radius{ -1.f };
	};

	struct PhysicsObj
	{
		float3D pos, vel; // pos and velocity of the Object.
		float3D rotationAxis;
		float rotationSpeed;
		float orientation;
		Matrix3 transformation;
		bool Colliding;
		AABB mAABB{};
		OBB mOBB{};
		Sphere mSphere{};
		float slantAngle; // rotation angle around orientation axis to orient the object in WCS.
		PhysicsObj(float3D p, float3D v, float rotS = 0.f, float rot = 0.f) :
			pos(p),
			vel(v),
			Colliding(false),
			rotationAxis({ 0.f, 0.f, 1.f }),
			rotationSpeed(rotS),
			slantAngle(rot),
			orientation(rot)
		{
		}
		float distSq(PhysicsObj const& obj) const {
			return (pos - obj.pos).LengthSq();
		};

		void setSphere()
		{
			std::cout << "setSphere: to be implemented by students using PCA method from Chapte 8\n";
		}
		void setAABB(float3D* vert, size_t n)
		{
			std::cout << "setAABB: to be implemented by students using PCA method from Chapte 8\n";
		}
		void setOBB()
		{
			std::cout << "setOBB: to be implemented by students using PCA method from Chapte 8\n";

		}
	};


	///////////////////// collision stuff ////////////////////
	struct CollisionData
	{
		CollisionData(shared_ptr<PhysicsObj> a, shared_ptr<PhysicsObj> b) {
			pair[0] = a;
			pair[1] = b;
		}
		float3D pt, MTD;  // MTD is minimum translation distance vector. pt is point of contact.
		shared_ptr<PhysicsObj> pair[2];
	};
	vector<shared_ptr<CollisionData>> CollisionDataList;
	////////////////////////////////////////////////////////

	// n sided polygons. It has a center and n vertices around it at average distance d from the center.
	struct Polygon : PhysicsObj
	{ // center position for a polygon is center of its OBB 
		size_t nSides;
		Vector3d<unsigned> color;
		float3D* vertices;
		bool* colliding;
		float3D* begin() { return &vertices[0]; }
		float3D* end() { return vertices + nSides; }
		void setVertex(unsigned n, float3D v) {
			assert(n < nSides);
			vertices[n] = v;
		}

		void setAABB()
		{
			PhysicsObj::setAABB(vertices, nSides);
		}

		Polygon(float3D c, float3D v, unsigned n, float rotS = 0.f, float rot = 0.f) : PhysicsObj(c, v, rotS, rot), nSides(n)
		{
			slantAngle = { rot };
			vertices = { new float3D[n] };
			colliding = { new bool[n] };
			memset((void*)vertices, 0, sizeof(float3D) * n);
		}

		// used to construct OBB for our polygon
		void computePrinicipleAxis()
		{
			// calculate the average:
			float3D mean{};
			std::for_each(begin(), end(), [&](auto p) {mean += p; });
			std::cout << "m:(" << mean.x << ", " << mean.y << std::endl;
			mean = { mean / nSides };
			
			//calculate covariance matrix:
			Matrix3 covMat{ Matrix3::zero() };
			// compute covariance matrix here:
			std::cout << "Compute covariance matrix by students\n";

			double covMatrixArray[3][3] =
			{
				{covMat.getRow(0).x, covMat.getRow(0).y, covMat.getRow(0).z },
				{covMat.getRow(1).x, covMat.getRow(1).y, covMat.getRow(1).z },
				{covMat.getRow(2).x, covMat.getRow(2).y, covMat.getRow(2).z },
			};

			double eigenVecs[3][3]{};
			double eigenVals[3]{};
			eigen_decomposition(covMatrixArray, eigenVecs, eigenVals);

			// provide code here to extract R, S, T and corresponding eigenValue

			float3D R{}, S{}, T{};
			// testing eigenValues:
			assert(eigenVals[2] >= eigenVals[1]);
			assert(eigenVals[1] >= eigenVals[0]);

			// finally here we compute extend and Center and Orient for our OBB
			Matrix3 orient{};
			orient.setRow(0, R);
			orient.setRow(1, S);
			orient.setRow(2, T);
			float3D extend{}; // students set this from above
			float3D center{};  // students set this from abov results
			mOBB.set(center, extend, orient);
		}
		~Polygon()
		{
			delete[] vertices;
			delete[] colliding;
		}
	};
	vector<shared_ptr<Polygon> > sPolygons;

	//////////////////////////////////////////////////////////////////////////////////////
	void DrawCircle(Polygon const& poly)
	{
		float3D cen{ poly.pos + poly.mSphere.center }; // cen in global coordinate system
		float cx{ cen.x};
		float cy{ cen.y };
		float theta = 2.f * PI / float(num_segments);
		float c = cos(theta);//precalculate the sine and cosine
		float s = sin(theta);
		float x = cos(poly.slantAngle) * poly.mSphere.radius;
		float y = sin(poly.slantAngle) * poly.mSphere.radius;
		float t = x;
		//	glMatrixMode(GL_MODELVIEW);
		//	glPushMatrix();
		//	glLoadIdentity();
		//	glTranslatef(ball.pos.x, ball.pos.y, ball.pos.z);
		//	glRotatef(ball.rotation, ball.rotationAxis.x, ball.rotationAxis.y, ball.rotationAxis.z);
		glColor3ub(poly.color.x, poly.color.y, poly.color.z);
		glBegin(GL_LINE_LOOP);
		for (size_t i = 0; i < num_segments; i++)
		{
			if (i % 2)
				glColor3ub(poly.color.x, poly.color.y, poly.color.z);
			else
				glColor3ub(10, 250, 250);
			glVertex2f(x + cx, y + cy);
			//apply the rotation matrix
			t = x;
			x = c * x - s * y;
			y = s * t + c * y;
		}
		glEnd();
		//	glPopMatrix();
	}

	////////////////////////////////////////////////////////////////////////////////////////////////////////
	void DrawSolidCircle(Polygon const& poly)
	{
		float3D cen{ poly.pos + poly.mSphere.center }; // cen in global coordinate system
		float cx{ cen.x };
		float cy{ cen.y };
		float theta = 2.f * PI / float(num_segments);
		float c{ cos(theta) };//precalculate the sine and cosine
		float s{ sin(theta) };
		glColor3ub(poly.color.x, poly.color.y, poly.color.z);
		float x = cos(poly.slantAngle) * poly.mSphere.radius;
		float y = sin(poly.slantAngle) * poly.mSphere.radius;
		float t{ x };
		glBegin(GL_TRIANGLE_FAN);
		glVertex2d(cx, cy);
		for (size_t i = 0; i < num_segments; i++)
		{
			//if(i%2)
			glColor3ub(poly.color.x, poly.color.y, poly.color.z);
			//else
			//	glColor3ub(10, 10, 250);

			glVertex2f(x + cx, y + cy);//output vertex 
			//apply the rotation matrix
			t = x;
			x = c * x - s * y;
			y = s * t + c * y;
		}

		glVertex2f(cos(poly.slantAngle) * poly.mSphere.radius + cx, cy);
		glEnd();
	}
	//////////////////////////////////////////////////////////////
	void DrawAABB(Polygon const& poly)
	{
		glLineWidth(3);
		glPointSize(7);
		glColor3ub(0, 0, 123);
		float3D ll = poly.pos + poly.mAABB.LL();
		float3D ur = poly.pos + poly.mAABB.UR();

		glBegin(GL_LINE_LOOP);
			glVertex2d(ll.x, ll.y);
			glVertex2d(ur.x, ll.y);
			glVertex2d(ur.x, ur.y);
			glVertex2d(ll.x, ur.y);
		glEnd();

		glPointSize(7);
		glColor3ub(0, 255, 0);
		// now render the center of AABB
		glBegin(GL_POINTS);
		float3D pt = poly.pos + poly.mAABB.pos; 
		glVertex2d(pt.x, pt.y);
		glEnd();

	}

	void DrawOBB(Polygon const& poly)
	{
		glLineWidth(3);
		glPointSize(7);
		glColor3ub(0, 0, 123);

		float3D offset{ poly.pos + poly.mOBB.center };
		float3D ext{ poly.mOBB.extend };
		const float3D& a{ poly.mOBB.orientation.getRow(0) };
		const float3D& b{ poly.mOBB.orientation.getRow(1) };
		
		float3D v1{ offset + a * ext.x + b * ext.y };
		float3D v2{ offset - a * ext.x + b * ext.y };
		float3D v3{ offset - a * ext.x - b * ext.y };
		float3D v4{ offset + a * ext.x - b * ext.y };

		glBegin(GL_LINE_LOOP);
		glVertex2f(v1.x, v1.y);
		glVertex2f(v2.x, v2.y);
		glVertex2f(v3.x, v3.y);
		glVertex2f(v4.x, v4.y);
		glEnd();
	}

	void DrawPolygon(Polygon& poly)
	{
		glColor3ub(poly.color.x, poly.color.y, poly.color.z);
		glLineWidth(3);
		glPointSize(7);
		glBegin(GL_LINES);
		for (int j = 0; j < poly.nSides; j++)
		{
			if(poly.colliding[j])
				glColor3ub(0, 255, 0);
			else
				glColor3ub(0, 0, 255);

			float3D pt1 = poly.pos + poly.transformation * poly.vertices[j];
			float3D pt2 = poly.pos + poly.transformation * poly.vertices[(j + 1) % poly.nSides];
			glVertex2d(pt1.x, pt1.y);
			glVertex2d(pt2.x, pt2.y);
		}
		glEnd();
		glPointSize(7);
		glColor3ub(0, 255, 0);
		glBegin(GL_POINTS);
		float3D pt = poly.pos;
		glVertex2d(pt.x, pt.y);
		glEnd();

	}
	//////////////////////////////////////////////////////////
	void DrawCollisionGeo(Polygon& poly)
	{
		glColor3ub(poly.color.x, poly.color.y, poly.color.z);
		switch (CollisionType)
		{
		case eCollisionMode::eSphere:
			DrawCircle(poly);
			break;
		case eCollisionMode::eAABB:
			DrawAABB(poly);
			break;
		case eCollisionMode::eOBB:
			DrawOBB(poly);
			break;
		}

		glBegin(GL_POLYGON);
		//glVertex2d(sPolygons[i].pos.x, sPolygons[i].pos.y);
		for (size_t j = 0; j < poly.nSides; j++)
		{
			float3D pt = poly.pos + poly.transformation * poly.vertices[j];
			glVertex2d(pt.x, pt.y);
		}
		glEnd();

	}
	//////////////////////////////////////////////////////////////////////////////////////////////
	bool doResolve{ false }; // 
	int Menu(void)
	{
		int r = eIdle, key;
		doResolve = true; // by default we do resolve collisions
		while (r != eStop && r != eStart)
		{
			FsPollDevice();
			key = FsInkey();
			switch (key)
			{
			case FSKEY_G:
				r = eStart;
				break;
			case FSKEY_ESC:
				r = eStop;
				break;
			case FSKEY_UP:
				iSpeed += 20.f;
				break;
			case FSKEY_DOWN:
				iSpeed = max(5.f, iSpeed - 20.f);
				break;
			case FSKEY_S:
				CollisionType = eCollisionMode::eSphere;
				break;
			case FSKEY_A:
				CollisionType = eCollisionMode::eAABB;
				break;
			case FSKEY_O:
				CollisionType = eCollisionMode::eOBB;
				break;
			case FSKEY_N:
				doResolve = false;
				break;
			}

			int wid, hei;
			FsGetWindowSize(wid, hei);

			glViewport(0, 0, wid, hei);
			glMatrixMode(GL_PROJECTION);
			glLoadIdentity();
			glOrtho(-0.5, (GLdouble)wid - 0.5, (GLdouble)hei - 0.5, -0.5, -1, 1);

			glClearColor(0.0, 0.0, 0.0, 0.0);
			glClear(GL_COLOR_BUFFER_BIT);
			char sSpeed[128];
			sprintf(sSpeed, "Average speed is %f m/s. Use Up/Down keys to change it!\n", iSpeed);
			char sPolyCnt[128];
			sprintf(sPolyCnt, "Polygon count is %d. Use Right/Left keys to change it!\n", PolyCount);
			char sCollision[255];
			sprintf(sCollision, "Collision detection is %s: A for AABB, S for Sphere, O for OBB\n",
				CollisionType == eCollisionMode::eSphere ? "Sphere Collision" :
				CollisionType == eCollisionMode::eAABB ? "AABB Colliion" : "OBB Collision");

			char sCollisionResolve[128];
			sprintf(sCollisionResolve, "Collisions are resolved? %s: (N for Not Resolving)\n", doResolve ? "Yes" : "No");

			glColor3ub(255, 255, 255);

			glRasterPos2i(32, 32);
			glCallLists(strlen(sSpeed), GL_UNSIGNED_BYTE, sSpeed);
			glRasterPos2i(32, 64);
			glCallLists(strlen(sCollision), GL_UNSIGNED_BYTE, sCollision);
			glRasterPos2i(32, 128);
			glCallLists(strlen(sPolyCnt), GL_UNSIGNED_BYTE, sPolyCnt);
			glRasterPos2i(32, 164);
			glCallLists(strlen(sCollisionResolve), GL_UNSIGNED_BYTE, sCollisionResolve);
		//	glRasterPos2i(32, 192);
		//	glCallLists(strlen(sResolveType), GL_UNSIGNED_BYTE, sResolveType);

			const char* msg1 = "G.....Start Game\n";
			const char* msg2 = "ESC...Exit";
			glRasterPos2i(32, 224);
			glCallLists(strlen(msg1), GL_UNSIGNED_BYTE, msg1);
			glRasterPos2i(32, 256);
			glCallLists(strlen(msg2), GL_UNSIGNED_BYTE, msg2);

			FsSwapBuffers();
			FsSleep(10);
		}

		// create polys
		if (r == eStart)
		{
			CollisionDataList.clear();
			CollisionDataList.reserve(PolyCount);

			sPolygons.clear();
			sPolygons.reserve(PolyCount);
		}
		return r;
	}


	//////////////////////////////////////////////////////////////
	void spheresCollisionCheck()
	{
		for_each(sPolygons.begin(), sPolygons.end(), [&](auto& p) {p->Colliding = false; });
		CollisionDataList.clear();
		for (int i = 0; i < sPolygons.size(); i++)
		{
			Polygon& poly1 = *sPolygons[i];
			for (int j = i + 1; j < sPolygons.size(); j++)
			{
				Polygon& poly2 = *sPolygons[j];
				if (poly1.distSq(poly2) < (poly1.mSphere.radius + poly2.mSphere.radius) * (poly1.mSphere.radius + poly2.mSphere.radius))
				{
					CollisionDataList.push_back(make_shared<CollisionData>(sPolygons[i], sPolygons[j]));
					float3D mtd{ poly2.mSphere.center + poly2.pos - (poly1.mSphere.center + poly1.pos) };
					mtd = mtd / mtd.Length();
					CollisionDataList.back()->MTD = (poly1.mSphere.radius + poly2.mSphere.radius - (float)((poly1.mSphere.center - poly2.mSphere.center)).Length() )* mtd;
					poly1.Colliding = poly2.Colliding = true;
				}
			}
		}
	}
	
	bool aabbCollide(Polygon const& poly1, Polygon const& poly2)
	{
		return false;
	}
	void aabbCollisionCheck()
	{
		for_each(sPolygons.begin(), sPolygons.end(), [&](auto& p) {p->Colliding = false; });
		CollisionDataList.clear();
		for (int i = 0; i < sPolygons.size(); i++)
		{
			Polygon& poly1 = *sPolygons[i];
			for (int j = i + 1; j < sPolygons.size(); j++)
			{
				Polygon& poly2 = *sPolygons[j];
				if (aabbCollide(poly1, poly2))
				{
				}
			}
		}
	}

	// THis assumes poly1 and poly2 are OBB. We fix this assumtion later when we generate OBB for any polygon using PCA
	// returns MTD for the 2 polys. If no overlap, then it returns zero length MTD
	float3D oBBCollideSAT(Polygon const& poly1, Polygon const& poly2)
	{
		// get 4 directions to check projections against:
		float4D dirs[4]{};

		// project polys on dirs, one by one, and check if they don't overlap then no collision ==> return false
		float overlap{FLT_MAX};
		int idx{};  
		for (int i = 0; i < 4; ++i)
		{
		}

		return float3D{};  // means they collide
	}

	// This is called when CollisionType is OBB
	void obbCollisionCheck()
	{
		for_each(sPolygons.begin(), sPolygons.end(), [&](auto& p) {p->Colliding = false; });
		CollisionDataList.clear();
		for (int i = 0; i < sPolygons.size(); i++)
		{
			Polygon& poly1 = *sPolygons[i];
			for (int j = i + 1; j < sPolygons.size(); j++)
			{
				Polygon& poly2 = *sPolygons[j];
				float3D mtd{ oBBCollideSAT(poly1, poly2) };
				if (mtd.LengthSq() > FLT_EPSILON)
				{
					CollisionDataList.push_back(make_shared<CollisionData>(sPolygons[i], sPolygons[j]));
					CollisionDataList.back()->MTD = mtd;
					poly1.Colliding = poly2.Colliding = true;
				}
			}
		}
	}

	int orientation(float3D p, float3D q, float3D r)
	{
		// See https://www.geeksforgeeks.org/orientation-3-ordered-points/
		// for details of below formula.
		int val = (q.y - p.y) * (r.x - q.x) -
			(q.x - p.x) * (r.y - q.y);

		if (val == 0) return 0;  // collinear

		return (val > 0) ? 1 : 2; // clock or counterclock wise
	}

	bool onSegment(float3D p, float3D q, float3D r)
	{
		if (q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x) &&
			q.y <= max(p.y, r.y) && q.y >= min(p.y, r.y))
			return true;

		return false;
	}
	
	bool Intersect(float3D p1, float3D q1, float3D p2, float3D q2)
	{
		// Find the four orientations needed for general and
		// special cases
		int o1 = orientation(p1, q1, p2);
		int o2 = orientation(p1, q1, q2);
		int o3 = orientation(p2, q2, p1);
		int o4 = orientation(p2, q2, q1);

		// General case
		if (o1 != o2 && o3 != o4)
			return true;

		// Special Cases
		// p1, q1 and p2 are collinear and p2 lies on segment p1q1
		if (o1 == 0 && onSegment(p1, p2, q1)) return true;

		// p1, q1 and q2 are collinear and q2 lies on segment p1q1
		if (o2 == 0 && onSegment(p1, q2, q1)) return true;

		// p2, q2 and p1 are collinear and p1 lies on segment p2q2
		if (o3 == 0 && onSegment(p2, p1, q2)) return true;

		// p2, q2 and q1 are collinear and q1 lies on segment p2q2
		if (o4 == 0 && onSegment(p2, q1, q2)) return true;

		return false; // Doesn't fall in any of the above cases
	}
	void segmentCollisionCheck()
	{
		bool do_collide = false;
		for_each(sPolygons.begin(), sPolygons.end(), [&](auto& p) {p->Colliding = false; });
		CollisionDataList.clear();
		for (int i = 0; i < sPolygons.size(); i++)
		{
			Polygon& poly = *sPolygons[i];
			for (size_t s1 = 0; s1 < poly.nSides; s1++)
			{
				poly.colliding[s1] = false;
			}
		}
		for (int i = 0; i < sPolygons.size(); i++)
		{

			Polygon& poly1 = *sPolygons[i];
			for (size_t s1 = 0; s1 < poly1.nSides; s1++)
			{
				float3D p1 = poly1.pos + poly1.transformation * poly1.vertices[s1];
				float3D q1 = poly1.pos + poly1.transformation * poly1.vertices[(s1 + 1) % poly1.nSides];


				for (int j = i + 1; j < sPolygons.size(); j++)
				{
					Polygon& poly2 = *sPolygons[j];
					poly2.Colliding = false;

					for (size_t s2 = 0; s2 < poly2.nSides; s2++)
					{
						float3D p2 = poly2.pos + poly2.transformation * poly2.vertices[s2];
						float3D q2 = poly2.pos + poly2.transformation * poly2.vertices[(s2 + 1) % poly2.nSides];

						if (Intersect(p1, q1, p2, q2))
						{
							// poly2.Colliding = true;

							poly1.colliding[s1] = true;
							poly2.colliding[s2] = true;

							do_collide = true;

							// poly1.Colliding = true;
						}
					}
				}
			}
		}

		pointSet = false;
		normalSet = false;
		if (do_collide)
		{

			for (int i = 0; i < sPolygons.size(); i++)
			{
				int total_colliding = 0;
				std::vector<size_t> points;

				Polygon& poly = *sPolygons[i];
				for (size_t s1 = 0; s1 < poly.nSides; s1++)
				{
					if (poly.colliding[s1])
					{
						++total_colliding;
						points.push_back(s1);
					}
				}
				if (1 == total_colliding)
				{
					// normal = <-
					float3D segment = ( poly.pos + poly.transformation * poly.vertices[(points[0] + 1) % poly.nSides] )
									- ( poly.pos + poly.transformation * poly.vertices[points[0]]					  );
					normal = float3D(-segment.y, segment.x);
					normal /= normal.Length();
					normalSet = true;
				}
				if (2 == total_colliding)
				{
					if ((points[0] + 1) == points[1])
					{
						pointSet = true;
						contactPoint = poly.pos + poly.transformation * poly.vertices[points[1]];
					}
					else if(poly.colliding[poly.nSides - 1] && poly.colliding[0])
					{
						pointSet = true;
						contactPoint = poly.pos + poly.transformation * poly.vertices[0];
					}
				}
			}

			if (normalSet && pointSet)
			{
				static float size_normal = 10.f;
				if (normalSet && pointSet)
				{
					float3D pt1 = contactPoint;
					float3D pt2 = contactPoint + normal * size_normal;
					//float3D pt1(50, 50);
					//	float3D pt2(60, 60);

					//glLineWidth(1);
					glBegin(GL_LINES);
					glColor3ub(0, 0, 0);

					glVertex2d(pt1.x, pt1.y);
					glVertex2d(pt2.x, pt2.y);

					glEnd();
				}

				if (doResolve) {
				float3D v1_prime = -sPolygons[0]->vel;
				float3D v2_prime = -sPolygons[1]->vel;

				float w1_prime = -sPolygons[0]->rotationSpeed;
				float w2_prime = -sPolygons[1]->rotationSpeed;

				sPolygons[0]->vel = v1_prime;
				sPolygons[1]->vel = v2_prime;

				sPolygons[0]->rotationSpeed = w1_prime;
				sPolygons[1]->rotationSpeed = w2_prime;
			}
			}
		}
	}


	void collisionResolve()
	{
	}
	/////////////////////////////////////////////////////////////////////
	void updatePhysics(double timeInc)
	{
		////////////Next update Polys positions //////////////////
		for_each(sPolygons.begin(), sPolygons.end(), [timeInc](shared_ptr<Polygon>& obj) {
			if(!pause){
				obj->pos += obj->vel * timeInc;
				obj->orientation += obj->rotationSpeed * timeInc;
			}
			obj->transformation.setRow(0, float3D(  cos(obj->orientation), sin(obj->orientation), 0));
			obj->transformation.setRow(1, float3D(- sin(obj->orientation), cos(obj->orientation), 0));
			obj->transformation.setRow(2, float3D(  0, 0, 1.f));
			});

		///// next check collisions ///////////////////////
		switch (CollisionType)
		{
		case eCollisionMode::eSphere:
			spheresCollisionCheck();
			segmentCollisionCheck(); // FIXME
			break;
		case eCollisionMode::eAABB:
			aabbCollisionCheck();
			break;
		case eCollisionMode::eOBB:
			obbCollisionCheck();
			break;
		}

		////// here you do collision resolution. /////////////
		if (doResolve)
		{
			//std::cout << "Resolving not done!\n";
			collisionResolve();
		}
	}


	//////////////////////////////////////////////////////////////////
	void checkEdgeCollision()
	{
		/////////////////////check polygons edge collision ////////////////////////////////////////
		for (auto& poly : sPolygons)
		{
			if (poly->pos.x < poly->mSphere.radius && poly->vel.x < 0) //checking left wall
			{
				poly->vel.x = -poly->vel.x;
			}
			if (poly->pos.y < poly->mSphere.radius && poly->vel.y < 0) // checking top wall
			{
				poly->vel.y = -poly->vel.y;
			}
			if (poly->pos.x > (WinWidth - poly->mSphere.radius) && poly->vel.x > 0) // checking right wall
			{
				poly->vel.x = -poly->vel.x;
			}
			if (poly->pos.y > (WinHeight - poly->mSphere.radius) && poly->vel.y > 0) // check bottom wall
			{
				poly->vel.y = -poly->vel.y;
			}
		}
	}

	//////////////////////////////////////////////////////
	void renderScene()
	{
		////// render polygons ///////////////
		for (auto& poly : sPolygons)
		{
			//		DrawCircle(Ball(poly->pos, poly->vel, poly->radius, poly->rotationSpeed, poly->rotation));
			if (!(*poly).Colliding)
				DrawPolygon(*poly);
			else
				DrawCollisionGeo(*poly);
		}

		////  swap //////////
		FsSwapBuffers();
	}

	//////////////////////////////////////////////////////////////
	void initPhysics()
	{
		int width = 0, height = 0;
		std::srand(std::time(0)); /* seed random number generator */

		FsGetWindowSize(width, height);

		int xdist = width / PolyCount;
		int ydist = height / PolyCount;

		for (int i = 0; i < PolyCount; ++i)
		{
			auto const &polydef(defpolys[i]);
			float rad = radius * (1. + float(std::rand() % PolyCount) / float(PolyCount));
			unsigned nSides = polydef.vertPolarCoordinate.size();
			sPolygons.push_back(make_shared<Polygon>(polydef.pos, polydef.vel, polydef.vertPolarCoordinate.size(), polydef.rotationVelocity));
			for (int j = 0; j < nSides; j++)
			{
				pair<float, float> const &vert(polydef.vertPolarCoordinate[j]);

				const float angle	 (vert.first );
				const float curRadius(vert.second);

				float3D side(curRadius * cos(angle), curRadius * sin(angle));
				sPolygons[i]->setVertex(&vert - &polydef.vertPolarCoordinate[0], side);
				auto obj(sPolygons[i]);
				obj->transformation.setRow(0, float3D( cos(obj->orientation), sin(obj->orientation), 0));
				obj->transformation.setRow(1, float3D(-sin(obj->orientation), cos(obj->orientation), 0));
				obj->transformation.setRow(2, float3D(0, 0, 1.f));

			}
			sPolygons[i]->color = { (unsigned)std::rand() % 250, (unsigned)std::rand() % 250, (unsigned)std::rand() % 250 };
			sPolygons[i]->setSphere();
			sPolygons[i]->setAABB();
			sPolygons[i]->computePrinicipleAxis();
		}
	}

	//////////////////////////////////////////////////////////////////////////////
	int Game(void)
	{
		DWORD passedTime = 0;
		FsPassedTime(true);

		int width = 0, height = 0;
		FsGetWindowSize(width, height);
		WinWidth = width;
		WinHeight = height;
		//////////// setting up the scene ////////////////////////////////////////	
		initPhysics();

		glViewport(0, 0, WinWidth, WinHeight);
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glOrtho(-0.5, (GLdouble)WinWidth - 0.5, (GLdouble)WinHeight - 0.5, -0.5, -1, 1);

		glClearColor(1.0, 1.0, 1.0, 1.0);
		glClear(GL_COLOR_BUFFER_BIT);

		////////////////////// main simulation loop //////////////////////////
		while (1)
		{
			glClear(GL_COLOR_BUFFER_BIT);
			int lb, mb, rb, mx, my;

			FsPollDevice();
			FsGetMouseState(lb, mb, rb, mx, my);
			int key = FsInkey();
			if (key == FSKEY_ESC)
				break;
			if (key == FSKEY_P)
				pause = !pause;

			checkEdgeCollision();

			////// update time lapse /////////////////
			passedTime = FsPassedTime();
			int timediff = timeSpan - passedTime;

			/////////// update physics /////////////////
			int maxPossible_dt = 2;
			int numOfIterations = timediff / maxPossible_dt + 1;	// Calculate Number Of Iterations To Be Made At This Update Depending On maxPossible_dt And dt
			double inc = (double)(timediff) / (double)numOfIterations * 0.001;
			for (int i = 0; i < numOfIterations; i++)
				updatePhysics(inc);

			renderScene();

			//	printf("\inc=%f, numOfIterations=%d, timediff=%d", inc, numOfIterations, timediff);
			while (timediff >= timeSpan / 3)
			{
				FsSleep(1);
				passedTime = FsPassedTime();
				timediff = timeSpan - passedTime;
				//	printf("--passedTime=%d, timediff=%d", passedTime, timediff);
			}
			passedTime = FsPassedTime(true);
		}
		return 0;
	}

	////////////////////////////////////////////////////////////////
	void GameOver(int score)
	{
		int r = 0;

		FsPollDevice();
		while (FsInkey() != 0)
		{
			FsPollDevice();
		}

		while (FsInkey() == 0)
		{
			FsPollDevice();

			int wid, hei;
			FsGetWindowSize(wid, hei);

			glViewport(0, 0, wid, hei);
			glMatrixMode(GL_PROJECTION);
			glLoadIdentity();
			glOrtho(0, (float)wid - 1, (float)hei - 1, 0, -1, 1);

			glClearColor(0.0, 0.0, 0.0, 0.0);
			glClear(GL_COLOR_BUFFER_BIT);

			const char* msg1 = "Game Over";
			char msg2[256];
			glColor3ub(255, 255, 255);
			glRasterPos2i(32, 32);
			glCallLists(strlen(msg1), GL_UNSIGNED_BYTE, msg1);

			sprintf(msg2, "Your score is %d", score);

			glRasterPos2i(32, 48);
			glCallLists(strlen(msg2), GL_UNSIGNED_BYTE, msg2);

			FsSwapBuffers();
			FsSleep(10);
		}
	}
}

	/////////////////////////////////////////////////////////////////////////////////////////
	int main(void)
	{
		int menu;
		FsOpenWindow(32, 32, WinWidth, WinHeight, 1); // 800x600 pixels, useDoubleBuffer=1

		int listBase;
		listBase = glGenLists(256);
		YsGlUseFontBitmap8x12(listBase);
		glListBase(listBase);

		glDisable(GL_DEPTH_TEST);
		glDisable(GL_LIGHTING);
		glDepthFunc(GL_ALWAYS);

		while (1)
		{
			menu = Menu();
			if (menu == eStart)
			{
				int score;
				score = Game();
				GameOver(score);
			}
			else if (menu == eStop)
			{
				break;
			}
		}

		return 0;
	}

