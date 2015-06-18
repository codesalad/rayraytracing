#include <stdio.h>
#ifdef WIN32
#include <windows.h>
#include <string>
#endif
#include <GL/glut.h>
#include "raytracing.h"
#include "Vec3D.h"
#include <vector>
#include <cfloat>
#include <list>
#include <limits>

using namespace std;
//temporary variables
//these are only used to illustrate 
//a simple debug drawing. A ray 
Vec3Df testRayOrigin;
Vec3Df testRayDestination;
float LightPos[3] = { 0, 0, 0 };
int selectedLight = 0;

// bounding box values
float xmin = std::numeric_limits<float>::max();
float xmax = std::numeric_limits<float>::min();
float ymin = std::numeric_limits<float>::max();
float ymax = std::numeric_limits<float>::min();
float zmin = std::numeric_limits<float>::max();
float zmax = std::numeric_limits<float>::min();

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


	// wchar_t buffer[MAX_PATH];
	// GetModuleFileName(NULL, buffer, MAX_PATH);
	// std::wstring::size_type pos = std::wstring(buffer).find_last_of(L"\\/");
	// std::wstring path = std::wstring(buffer).substr(0, pos + 1);
	// path += L"blocks.obj";
	// std::string res(path.begin(), path.end());
	// printf(res.c_str());

    MyMesh.loadMesh("cornell.obj", true);
	MyMesh.computeVertexNormals();
	std::vector<Vertex> vertices = MyMesh.vertices;
	
	//Establish values for bounding box	
	for (int i = 0; i < vertices.size(); ++i) {
	   
	    if (vertices[i].p[0] < xmin) {
			xmin = vertices[i].p[0];
	    }
	    if (vertices[i].p[0] > xmax) {
			xmax = vertices[i].p[0];
	    }
	    if (vertices[i].p[1] < ymin) {
			ymin = vertices[i].p[1];
	    }
	    if (vertices[i].p[1] > ymax) {
			ymax = vertices[i].p[1];
	    }
	    if (vertices[i].p[2] < zmin) {
			zmin = vertices[i].p[2];
	    }
	    if (vertices[i].p[2] > zmax) {
			zmax = vertices[i].p[2];
	    }
	}
	
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
vector<float> intersect(Vec3Df origin, Vec3Df dest) {
	std::vector<Triangle> triangles = MyMesh.triangles;
	Vec3D<float> intPoint;
	vector<float> intersectData;
	//list<Vec3D<float> > intersectedTriangles;
	vector<float> closestIntersect;
	Vec3D<float> origin3 = origin;
	Vec3D<float> dest3 = dest;
	float dMax = FLT_MAX;
	// Bounding box
	Vec3Df bborigin = origin;
	Vec3Df bbdest = dest;

	// Calculate intersection parameters for each plane.
	float txmin = (xmin - bborigin[0]) / bbdest[0];
	float txmax = (xmax - bborigin[0]) / bbdest[0];
	float tymin = (ymin - bborigin[1]) / bbdest[1];
	float tymax = (ymax - bborigin[1]) / bbdest[1];
	float tzmin = (zmin - bborigin[2]) / bbdest[2];
	float tzmax = (zmax - bborigin[2]) / bbdest[2];

	// Calculate which point is entry and which is exit.
	float tinx = min(txmin, txmax);
	float toutx = max(txmin, txmax);
	float tiny = min(tymin, tymax);
	float touty = max(tymin, tymax);
	float tinz = min(tzmin, tzmax);
	float toutz = max(tzmin, tzmax);
	float tin = max(tinx, tiny, tinz);
	float tout = min(toutx, touty, toutz);

	if (tin <= tout && tout > 0) { // The ray hits the bounding box, so we do the computations.
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
			float t = (D - Vec3D<float>::dotProduct(origin, normal)) / Vec3D<float>::dotProduct(dest, normal);

			Vec3D<float> origin2 = origin;
			Vec3D<float> dest2 = dest;
			// Finished product. intPoint is the intersection point.
			intPoint = origin2 + t*dest2;

			float D2 = Vec3Df().distance(intPoint, MyCameraPosition);

			if (D2 < dMax) {
				// vertex0 = static point, A.
				// v0, v1 still the 2 edges connected to vertex0.
				// v2 = P - A.
				Vec3D<float> v2;
				v2.init(intPoint.p[0] - vertex0.p[0], intPoint.p[1] - vertex0.p[1], intPoint.p[2] - vertex0.p[2]);

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
					intersectData.push_back(normal[0]);
					intersectData.push_back(normal[1]);
					intersectData.push_back(normal[2]);
					intersectData.push_back(i);
					closestIntersect = intersectData;
					dMax = D2;
				} 
			}
		}
		return closestIntersect;
	}
	else {
		vector<float> intersectData;
		return intersectData;
	}
}

/**
* Computes the direct color using the hitPoint and the triangleIndex.
* Returns RGB Vec3Df.
*/
Vec3Df computeDirectLight(Vec3Df& hitPoint, int& triangleIndex, const Vec3Df& dest, Vec3Df& normalIn)
{
	// Setting up mat indices
	std::vector<Triangle> triangles = MyMesh.triangles;
	std::vector<unsigned int> triangleMaterials = MyMesh.triangleMaterials;
	std::vector<Material> materials = MyMesh.materials;
	int triangleMatIndex = triangleMaterials.at(triangleIndex);
	Material mat = materials.at(triangleMatIndex);

	Vec3D<float> normal = normalIn;
	Vec3D<float> lightray = dest;
	Vec3D<float> hit = hitPoint;

	// Diffuse
	
	lightray.normalize();	// normalize.
	hit.normalize();


	float c = abs(Vec3D<float>::dotProduct(lightray, normal));

	Vec3Df diffuse = mat.Kd() * c;


	// Specular
	Vec3Df viewDirec = MyCameraPosition/MyCameraPosition.getLength();
	Vec3Df halfDirec = viewDirec - lightray;
	halfDirec.normalize();

	float angle = abs(Vec3D<float>::dotProduct(halfDirec,normal));


	Vec3Df specular = 1 *  mat.Ks() * pow(angle, mat.Ns());

	return (mat.Ka() + diffuse + specular);
}


/**
            Vector incomVector    = VectorSubtract(intersectionPosition, lightPosition);
            Vector incomVectorN   = VectorNormalize(incomVector);

            float myDot = - VectorDotProduct(incomVectorN, intersectionNormalN);
            float myLen = 2.0f * myDot;

            Vector tempNormal     = VectorMultScalar(intersectionNormalN, myLen);
            Vector reflectVector  = VectorAdd(tempNormal, incomVectorN);
            Vector reflectVectorN = VectorNormalize(reflectVector);

            float mySpec = MAX(-VectorDotProduct(reflectVectorN, incomVectorN), 0);
            mySpec       = powf(mySpec, 5);

            specularColor = ColorMultScalar(specularColor, mySpec);
            pixelColor    = ColorAdd(pixelColor, specularColor);
            pixelColor    = ColorClamp(pixelColor);
**/

//return the color of your pixel.
Vec3Df performRayTracing(const Vec3Df & origin, const Vec3Df & dest)
{
  vector<float> intersectData = intersect(origin, dest);
	//cout << intersectData.size() <<endl;
	if (intersectData.size() > 0 ) {
		Vec3Df hitPoint = Vec3Df(intersectData.at(0), intersectData.at(1), intersectData.at(2));
		Vec3Df normal = Vec3Df(intersectData.at(3), intersectData.at(4), intersectData.at(5));
		int triangleIndex = intersectData.back();
		Vec3Df colorRGB = computeDirectLight(hitPoint, triangleIndex, dest, normal);
		return Vec3Df(colorRGB[0], colorRGB[1], colorRGB[2]);
	}

	return Vec3Df(0,0,0);
}


void yourDebugDraw()
{
	//draw open gl debug stuff
	//this function is called every frame

	//let's draw the mesh
	//MyMesh.draw();
	MyMesh.drawSmooth();
	
	//let's draw the lights in the scene as points
	glPushAttrib(GL_ALL_ATTRIB_BITS); //store all GL attributes
	//glDisable(GL_LIGHTING);
	glEnable(GL_LIGHTING);
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
	//glDisable(GL_LIGHTING);
	glEnable(GL_LIGHTING);
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
	switch (t)
	{
	case 'w':
		LightPos[2] += 0.25;
		break;
	case 's':
		LightPos[2] -= 0.25;
		break;

	case 'a':
		LightPos[1] -= 0.25;
		break;
	case 'd':
		LightPos[1] += 0.25;
		break;

	case 't':
		LightPos[0] -= 0.25;
		break;
	case 'f':
		LightPos[0] += 0.25;
		break;

	case '0':
		if (MyLightPositions.size() > 0)
		{
			selectedLight = 0;
		}
		break;
	case '1':
		if (MyLightPositions.size() > 1)
		{
			selectedLight = 1;
		}
		break;
	case '2':
		if (MyLightPositions.size() > 2)
		{
			selectedLight = 2;
		}
		break;
	case '3':
		if (MyLightPositions.size() > 3)
		{
			selectedLight = 3;
		}
		break;
	case '4':
		if (MyLightPositions.size() > 4)
		{
			selectedLight = 4;
		}
		break;
	case '5':
		if (MyLightPositions.size() > 5)
		{
			selectedLight = 5;
		}
		break;
	case '6':
		if (MyLightPositions.size() > 6)
		{
			selectedLight = 6;
		}
		break;
	case '7':
		if (MyLightPositions.size() > 7)
		{
			selectedLight = 7;
		}
		break;
	case '8':
		if (MyLightPositions.size() > 8)
		{
			selectedLight = 8;
		}
		break;
	case '9':
		if (MyLightPositions.size() > 9)
		{
			selectedLight = 9;
		}
		break;
	}

	Vec3Df res = MyLightPositions[selectedLight];
	res[0] += LightPos[0];
	res[1] += LightPos[1];
	res[2] += LightPos[2];
	MyLightPositions[selectedLight] = res;

	//glutPostRedisplay();

	LightPos[0] = 0;
	LightPos[1] = 0;
	LightPos[2] = 0;


	//here, as an example, I use the ray to fill in the values for my upper global ray variable
	//I use these variables in the debugDraw function to draw the corresponding ray.
	//try it: Press a key, move the camera, see the ray that was launched as a line.
	testRayOrigin=rayOrigin;	
	testRayDestination=rayDestination;
	
	// do here, whatever you want with the keyboard input t.
	
	//...
	
	
	// std::cout<<t<<" pressed! The mouse was in location "<<x<<","<<y<<"!"<<std::endl;	
}

void Shading()
{
	/*
	use MyMesh.triangles
	MyMesh.triangleMaterials
	MyMesh.materials
	*/
	int selLight = 0;
	int selTriangle = 0;
	ComputeAmbient(selLight, selTriangle);
	ComputeDiffuse(selLight, selTriangle);
	ComputeSpecular(selLight, selTriangle);
}

Vec3Df ComputeAmbient(int selLight, int selTriangle)
{
	unsigned int trMaterialIndex = MyMesh.triangleMaterials[selTriangle];

	glEnable(GL_AMBIENT);
	Vec3Df mAmbient = MyMesh.materials[trMaterialIndex].Ka();
	float lAmbient =  1;
	return mAmbient * lAmbient;
}

Vec3Df ComputeDiffuse(int selLight, int selTriangle)
{
	Vec3Df normal;
	Vec3Df lightDir;
	Vec3Df diffuse;
	Vec3Df color;
	float dotProduct;

	unsigned int trMaterialIndex = MyMesh.triangleMaterials[selTriangle];
	Vec3Df mDiffuse = MyMesh.materials[trMaterialIndex].Kd();
	float lDiffuse = 1;
	
	//normal = normalize(gl_NormalMatrix * gl_Normal);**/
	glEnable(GL_NORMALIZE);
	// transform normal vector and then normalize
	Vertex vertex1 = MyMesh.vertices[MyMesh.triangles[selTriangle].v[0]];
	Vertex vertex2 = MyMesh.vertices[MyMesh.triangles[selTriangle].v[1]];
	normal = Vec3Df().crossProduct(vertex1.p, vertex2.p);
	normal.normalize();

    //lightDir = normalize(vec3(gl_LightSource[0].position));
	lightDir[0] = MyLightPositions[selLight][0] - MyMesh.triangles[selTriangle].v[0];
	lightDir[1] = MyLightPositions[selLight][1] - MyMesh.triangles[selTriangle].v[1];
	lightDir[2] = MyLightPositions[selLight][2] - MyMesh.triangles[selTriangle].v[2];
	lightDir.normalize();

	//NdotL = max(dot(normal, lightDir), 0.0);
	dotProduct = Vec3Df().dotProduct(normal, lightDir);
	if (dotProduct < 0)
	{
		dotProduct = dotProduct * (-1);
	}
	double cosinus = dotProduct;
	//diffuse = gl_FrontMaterial.diffuse * gl_LightSource[0].diffuse;
	diffuse = mDiffuse * lDiffuse * cosinus;

	//gl_FrontColor = NdotL * diffuse;
	color = diffuse;

	//gl_Position = ftransform();
	return color;
}

Vec3Df ComputeSpecular(int selLight, int selTriangle)
{
	Vec3Df normal;
	Vec3Df lightDir;
	Vec3Df specular;
	float dotProduct;
	float NormalizeHV;

	unsigned int trMaterialIndex = MyMesh.triangleMaterials[selTriangle];
	
	Vec3Df mSpecular = MyMesh.materials[trMaterialIndex].Ks();
	float lSpecular = 0.1;
	float mShininess = MyMesh.materials[trMaterialIndex].Ns();
	
	Vertex vertex1 = MyMesh.vertices[MyMesh.triangles[selTriangle].v[0]];
	Vertex vertex2 = MyMesh.vertices[MyMesh.triangles[selTriangle].v[1]];
	normal = Vec3Df().crossProduct(vertex1.p, vertex2.p);
	glEnable(GL_NORMALIZE);
	normal.normalize();

	Vec3Df viewPoint = MyCameraPosition;
	Vec3Df viewDir;
	viewDir[0] = viewPoint[0] - MyMesh.triangles[selTriangle].v[0];
	viewDir[1] = viewPoint[1] - MyMesh.triangles[selTriangle].v[1];
	viewDir[2] = viewPoint[2] - MyMesh.triangles[selTriangle].v[2];

	dotProduct = Vec3Df().dotProduct(normal, viewDir);
	if (dotProduct < 0)
	{
		dotProduct = dotProduct * (-1);
	}

	Vec3Df refl = viewDir - 2 * dotProduct * normal;
	refl.normalize();

	viewDir.normalize();
	float dotProduct2 = Vec3Df().dotProduct(refl, viewDir);
	if (dotProduct2 < 0)
	{
		dotProduct2 = dotProduct2 * (-1);
	}

	specular = mSpecular * lSpecular *	pow(cos(dotProduct2), mShininess);
	return specular;
}
/*
bool intersectBoundingBox(const Vec3Df & origin, const Vec3Df & dest)
{
	// bounding box
	Vec3Df bborigin = origin;
	Vec3Df bbdest = dest;

	float txmin = (xmin - bborigin[0]) / bbdest[0];
	float txmax = (xmax - bborigin[0]) / bbdest[0];
	float tymin = (ymin - bborigin[1]) / bbdest[1];
	float tymax = (ymax - bborigin[1]) / bbdest[1];
	float tzmin = (zmin - bborigin[2]) / bbdest[2];
	float tzmax = (zmax - bborigin[2]) / bbdest[2];

	float tinx = min(txmin, txmax);
	float toutx = max(txmin, txmax);
	float tiny = min(tymin, tymax);
	float touty = max(tymin, tymax);
	float tinz = min(tzmin, tzmax);
	float toutz = max(tzmin, tzmax);
	float tin = max(tinx, tiny, tinz);
	float tout = min(toutx, touty, toutz);
	if (tin > tout || tout < 0)
	{
		return false;
	}
	else
	{
		return true;
	}
}
*/

float min (float f1, float f2)
{
  if (f1 < f2)
    return f1;
  else 
    return f2;
}

float max (float f1, float f2)
{
  if (f1 > f2)
    return f1;
  else
    return f2;
}

float min (float f1, float f2, float f3)
{
  float m = f1;
  if (f2 < m) {
    m = f2;
  }
  if (f3 < m){
    m = f3;
  }
  return m;
}

float max (float f1, float f2, float f3)
{
  float m = f1;
  if (f2 > m)
    m = f2;
  if (f3 > m)
    m = f3;
  return m;
}
