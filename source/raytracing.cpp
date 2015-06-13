#include <stdio.h>
#ifdef WIN32
#include <windows.h>
#endif
#include <GL/glut.h>
#include "raytracing.h"
#include "Vec3D.h"
#include <vector>
#include <cfloat>
#include <list>

using namespace std;
//temporary variables
//these are only used to illustrate 
//a simple debug drawing. A ray 
Vec3Df testRayOrigin;
Vec3Df testRayDestination;


//use this function for any preprocessing of the mesh.
void init()
{
	//load the mesh file
	//please realize that not all OBJ files will successfully load.
	//Nonetheless, if they come from Blender, they should, if they 
	//are exported as WavefrontOBJ.
	//PLEASE ADAPT THE LINE BELOW TO THE FULL PATH OF THE dodgeColorTest.obj
	//model, e.g., "C:/temp/myData/GraphicsIsFun/dodgeColorTest.obj", 
	//otherwise the application will not load properly
    MyMesh.loadMesh("monkey.obj", true);
	MyMesh.computeVertexNormals();

	//one first move: initialize the first light source
	//at least ONE light source has to be in the scene!!!
	//here, we set it to the current location of the camera
	MyLightPositions.push_back(MyCameraPosition);
}

/**
* Tests whether the ray intersects with the triangle.
* Using the ray, calculates the hit points and returns it.
* Also checks if the hit point also lies within the plane using barycentric variables.
*/
vector<float> intersect(const Vec3Df & origin, const Vec3Df & dest)
{
	std::vector<Triangle> triangles = MyMesh.triangles;
	Vec3D<float> intPoint;
	vector<float> intersectData;
	//list<Vec3D<float> > intersectedTriangles;
	vector<float> closestIntersect;
	float dMax = FLT_MAX;
	for (int i = 0; i < triangles.size(); ++i) {
		// Initialize the 3 vertex points of the triangle.
		Vertex vertex0 = MyMesh.vertices.at(triangles.at(i).v[0]);
		Vertex vertex1 = MyMesh.vertices.at(triangles.at(i).v[1]);
		Vertex vertex2 = MyMesh.vertices.at(triangles.at(i).v[2]);

		// Calculate the two edges.
		// Vector v0 = Vector 1 - Vector 0.
		// Vecotr v1 = Vector 2 - Vector 0.
		Vec3D<float> v0;
		Vec3D<float> v1;
		v0.init(vertex1.p[0] - vertex0.p[0], vertex1.p[1] - vertex0.p[1], vertex1.p[2] - vertex0.p[2]);
		v1.init(vertex2.p[0] - vertex0.p[0], vertex2.p[1] - vertex0.p[1], vertex2.p[2] - vertex0.p[2]);
		
		// Calculate the distance plane to origin.
		// Using a vertex from the triangle, orthogonally project onto normal vector.
		Vec3D<float> normal = Vec3D<float>::crossProduct(v0, v1);
		// Saw in the slides... not sure if we need this?
		normal.normalize();
		if (Vec3D<float>::dotProduct(v0,normal) < 0) {
			normal = -normal;
		}

		Vec3D<float> vertexVector;
		vertexVector.init(vertex0.p[0], vertex0.p[1], vertex0.p[2]);
		float D = (Vec3D<float>::dotProduct(vertexVector, normal));

		float t = (D - Vec3D<float>::dotProduct(origin, normal))/Vec3D<float>::dotProduct(dest, normal);

		Vec3D<float> origin2 = origin;
		Vec3D<float> dest2 = dest;
		// Finished product. intPoint is the intersection point.
		intPoint = origin2 + t*dest2;

		// vertex0 = static point, A.
		// v0, v1 still the 2 edges connected to vertex0.
		// v2 = P - A.
		float dpaX = intPoint.p[0] - vertex0.p[0];
		float dpaY = intPoint.p[1] - vertex0.p[1];
		float dpaZ = intPoint.p[2] - vertex0.p[2];
		Vec3D<float> v2;
		v2.init(dpaX, dpaY, dpaZ);

		// These dot products are derived from the linear combinations of edges.
		// u and v are the barycentric coordinates.
		float dot00 = Vec3D<float>::dotProduct(v0, v0);
		float dot01 = Vec3D<float>::dotProduct(v0, v1);
		float dot02 = Vec3D<float>::dotProduct(v0, v2);
		float dot11 = Vec3D<float>::dotProduct(v1, v1);
		float dot12 = Vec3D<float>::dotProduct(v1, v2);

		float invDenom = 1 / (dot00 * dot11 - dot01 * dot01);
		float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
		float v = (dot00 * dot12 - dot01 * dot02) * invDenom;
		if ( (u >= 0) && (v >= 0) && (u + v < 1) ) {
			intersectData.push_back(intPoint.p[0]);
			intersectData.push_back(intPoint.p[1]);
			intersectData.push_back(intPoint.p[2]);
			intersectData.push_back(i);
			if (D < dMax) {
				closestIntersect = intersectData;
				dMax = D;
			}
		} 
	}
	return closestIntersect;
}

/**
* Computes the direct color using the hitPoint and the triangleIndex.
* Returns RGB Vec3Df.
*/
Vec3Df directColor(Vec3Df& hitPoint, int& triangleIndex)
{
	//Material triangleMaterial = MyMesh.materials(MyMesh.triangleMaterials(triangleIndex));
	std::vector<Triangle> triangles = MyMesh.triangles;
	std::vector<unsigned int> triangleMaterials = MyMesh.triangleMaterials;
	std::vector<Material> materials = MyMesh.materials;

	int triangleMatIndex = triangleMaterials.at(triangleIndex);
	Material mat = materials.at(triangleMatIndex);

	Vec3Df diffuse = mat.Kd();
	Vec3Df ambience = mat.Ka();

	return Vec3Df(diffuse[0]+ambience[0], diffuse[1]+ambience[1], diffuse[2]+ambience[2]);
}

//return the color of your pixel.
Vec3Df performRayTracing(const Vec3Df & origin, const Vec3Df & dest)
{
	vector<float> intersectData = intersect(origin, dest);
	//cout << intersectData.size() <<endl;
	if (intersectData.size() > 0 ) {
		Vec3Df hitPoint = Vec3Df(intersectData.at(0), intersectData.at(1), intersectData.at(2));
		int triangleIndex = intersectData.back();
		Vec3Df colorRBB = directColor(hitPoint, triangleIndex);
		return Vec3Df(colorRBB[0], colorRBB[1], colorRBB[2]);
	}

	return Vec3Df(.1,.1,.1);
}

void PutPixel(int& x, int& y, Vec3Df color)
{



}

void Shade(int level,Vec3Df hit, Vec3Df &color)
{
	/*Vec3Df directColor, reflectedRay, reflectedColor, refractedRay, refractedColor;

	ComputeDirectLight( hit, &directColor );
	ComputeReflectedRay( hit, &reflectedRay );
	Trace( level+1, reflectedRay, &reflectedColor );
	ComputeRefractedRay( hit, &refractedRay );
	Trace( level+1, refractedRay, &refractedColor );
	color = directColor + reflection * reflectedColor + transmission * refractedColor;*/
}

void ComputeDirectLight(Vec3Df hit, int &triangleIndex, Vec3Df &directColor )
{
	/*for(unsigned int i=0; i<MyLightPositions.size(); ++i){
		if( shadowtest(MylightPositions[i],hit) ){
			Material mat = MyMesh.materials[triangleMaterials[triangleIndex]];
			
			Vec3Df diff = mat.Kd;
			Vec3Df amb = mat.Ka;
			Vec3Df spec = mat.Ks;
		} 
	
	}	*/
}


void yourDebugDraw()
{
	//draw open gl debug stuff
	//this function is called every frame

	//let's draw the mesh
	MyMesh.draw();
	
	//let's draw the lights in the scene as points
	glPushAttrib(GL_ALL_ATTRIB_BITS); //store all GL attributes
	glDisable(GL_LIGHTING);
	glColor3f(1,1,1);
	glPointSize(10);
	glBegin(GL_POINTS);
	for (int i=0;i<MyLightPositions.size();++i)
		glVertex3fv(MyLightPositions[i].pointer());
	glEnd();
	glPopAttrib();//restore all GL attributes
	//The Attrib commands maintain the state. 
	//e.g., even though inside the two calls, we set
	//the color to white, it will be reset to the previous 
	//state after the pop.


	//as an example: we draw the test ray, which is set by the keyboard function
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glDisable(GL_LIGHTING);
	glBegin(GL_LINES);
	glColor3f(0,1,1);
	glVertex3f(testRayOrigin[0], testRayOrigin[1], testRayOrigin[2]);
	glColor3f(0,0,1);
	glVertex3f(testRayDestination[0], testRayDestination[1], testRayDestination[2]);
	glEnd();
	glPointSize(10);
	glBegin(GL_POINTS);
	glVertex3fv(MyLightPositions[0].pointer());
	glEnd();
	glPopAttrib();
	
	//draw whatever else you want...
	////glutSolidSphere(1,10,10);
	////allows you to draw a sphere at the origin.
	////using a glTranslate, it can be shifted to whereever you want
	////if you produce a sphere renderer, this 
	////triangulated sphere is nice for the preview
}


//yourKeyboardFunc is used to deal with keyboard input.
//t is the character that was pressed
//x,y is the mouse position in pixels
//rayOrigin, rayDestination is the ray that is going in the view direction UNDERNEATH your mouse position.
//
//A few keys are already reserved: 
//'L' adds a light positioned at the camera location to the MyLightPositions vector
//'l' modifies the last added light to the current 
//    camera position (by default, there is only one light, so move it with l)
//    ATTENTION These lights do NOT affect the real-time rendering. 
//    You should use them for the raytracing.
//'r' calls the function performRaytracing on EVERY pixel, using the correct associated ray. 
//    It then stores the result in an image "result.ppm".
//    Initially, this function is fast (performRaytracing simply returns 
//    the target of the ray - see the code above), but once you replaced 
//    this function and raytracing is in place, it might take a 
//    while to complete...
void yourKeyboardFunc(char t, int x, int y, const Vec3Df & rayOrigin, const Vec3Df & rayDestination)
{

	//here, as an example, I use the ray to fill in the values for my upper global ray variable
	//I use these variables in the debugDraw function to draw the corresponding ray.
	//try it: Press a key, move the camera, see the ray that was launched as a line.
	testRayOrigin=rayOrigin;	
	testRayDestination=rayDestination;
	
	// do here, whatever you want with the keyboard input t.
	
	//...
	
	
	std::cout<<t<<" pressed! The mouse was in location "<<x<<","<<y<<"!"<<std::endl;	
}
