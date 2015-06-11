#include <stdio.h>
#ifdef WIN32
#include <windows.h>
#endif
#include <GL/glut.h>
#include "raytracing.h"
#include "Vec3D.h"
#include <cfloat>

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
    MyMesh.loadMesh("dodgeColorTest.obj", true);
	MyMesh.computeVertexNormals();

	//one first move: initialize the first light source
	//at least ONE light source has to be in the scene!!!
	//here, we set it to the current location of the camera
	MyLightPositions.push_back(MyCameraPosition);
}

Vec3Df intersect(const Vec3Df & origin, const Vec3Df & dest)
{
	float t = FLT_MAX;
	std::vector<Triangle> triangles = MyMesh.triangles;
	for (int i = 0; i < 1; ++i) {
		int vertexIndex0 = triangles.at(i).v[0];
		int vertexIndex1 = triangles.at(i).v[1];
		int vertexIndex2 = triangles.at(i).v[2];

		Vertex vertex0 = MyMesh.vertices.at(vertexIndex0);
		Vertex vertex1 = MyMesh.vertices.at(vertexIndex1);
		Vertex vertex2 = MyMesh.vertices.at(vertexIndex2);

		// Vector X = Vector 1 - Vector 0
		// Vecotr Y = Vector 2 - Vector 0

		float dx1 = vertex1.p[0] - vertex0.p[0];
		float dy1 = vertex1.p[1] - vertex0.p[1];
		float dz1 = vertex1.p[2] - vertex0.p[2];

		float dx2= vertex2.p[0] - vertex0.p[0];
		float dy2 = vertex2.p[1] - vertex0.p[1];
		float dz2 = vertex2.p[2] - vertex0.p[2];

		Vec3D<float> x;
		Vec3D<float> y;

		x.init(dx1, dy1, dz1);
		y.init(dx2, dy2, dz2);
		
		Vec3D<float> crossProductxy = Vec3D<float>::crossProduct(x, y);

		Vec3D<float> normal = crossProductxy / crossProductxy.getLength();

		Vec3D<float> vertexVector;
		vertexVector.init(vertex0.p[0], vertex1.p[1], vertex2.p[2]);

		float D = (Vec3D<float>::dotProduct(vertexVector, normal) * normal).getLength();
		float t = (D - Vec3D<float>::dotProduct(origin, normal))/Vec3D<float>::dotProduct(dest, normal);
		
		Vec3D<float> origin2;
		origin2 = origin;

		Vec3D<float> dest2;
		dest2 = dest;

		Vec3D<float> p = origin2 + t*dest2;
		cout << "intersect: " << p << endl;
	}



}

//return the color of your pixel.
Vec3Df performRayTracing(const Vec3Df & origin, const Vec3Df & dest)
{
	intersect(origin, dest);

	return Vec3Df(dest[0],dest[1],dest[2]);
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
