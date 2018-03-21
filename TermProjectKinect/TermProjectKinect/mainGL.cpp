#include "stdafx.h"
#include "glut.h"
#include <glut.h>
#include <cmath>
#include <cstdio>
#include <Windows.h>
#include <time.h>
#include <iostream>
#include <Ole2.h>
#include <NuiApi.h>
#include <math.h>
#include <stdlib.h>
#include <NuiImageCamera.h>
#include <NuiSensor.h>
#define width 640
#define height 480

using namespace std;
// OpenGL Variables
long depthToRgbMap[width*height * 2];
// We'll be using buffer objects to store the kinect point cloud
GLuint vboId;
GLuint cboId;
int randX = 0, randY = 0, randZ = 0;
int x = 0, y = 0, z = 0;
double X, Y, Z;

static int 	timer = 1;
int hit = 0;

// Kinect variables
HANDLE depthStream;
HANDLE rgbStream;
INuiSensor* sensor;

//modeling functions
void createDigda(GLfloat centerx, GLfloat centery, GLfloat centerz, GLfloat radius, GLfloat h);
void createBody(GLfloat centerx, GLfloat centery, GLfloat centerz, GLfloat radius, GLfloat h,GLfloat color[3]);
void createHalfSphere(GLfloat x, GLfloat y, GLfloat z, GLfloat radius, GLfloat color[3]);
void DrawEllipse(float radiusX, float radiusY, GLfloat color[3]);
void DrawHit();
void drawBar(float size, int times);
void drawStick(float size, int times);
void drawHammer();

//static int delay = 50;
int hei = 10;
int change = 1;
double GL_PI = atan(1.0)*4.0;
#define DEG2RAD 3.14159/180.0

//light model
typedef struct {
	GLfloat pos[4];	// position
	GLfloat amb[4];	// ambient
	GLfloat dif[4];	// diffuse
	GLfloat spe[4];	// specular
} Light;

Light light = {
	{ 3.0, 3.0, 3.0, 1.0 },	// position
	{ 1.0, 1.0, 1.0, 1.0 }, // ambient
	{ 1.0, 1.0, 1.0, 1.0 }, // diffuse
	{ 1.0, 1.0, 1.0, 1.0 }, // specular
};


GLfloat Body[3] = { 185,122,87 }, nose[3] = { 255,128,128 }, eye[3] = { 64,0,0 }, stone[3] = { 128,128,128 };
static int delay = 0;
static int delay2 = 0;

// Stores the coordinates of each joint
Vector4 skeletonPosition[NUI_SKELETON_POSITION_COUNT];
//int delay = 0.1;
static double mov = 0.0;
static double lhX, lhY, lhZ;
bool initKinect() {
	// Get a working kinect sensor
	int numSensors;
	if (NuiGetSensorCount(&numSensors) < 0 || numSensors < 1) return false;
	if (NuiCreateSensorByIndex(0, &sensor) < 0) return false;

	// Initialize sensor
	sensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX | NUI_INITIALIZE_FLAG_USES_COLOR | NUI_INITIALIZE_FLAG_USES_SKELETON);
	sensor->NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX, // Depth camera or rgb camera?
		NUI_IMAGE_RESOLUTION_640x480,                // Image resolution
		0,        // Image stream flags, e.g. near mode
		2,        // Number of frames to buffer
		NULL,     // Event handle
		&depthStream);
	sensor->NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR, // Depth camera or rgb camera?
		NUI_IMAGE_RESOLUTION_640x480,                // Image resolution
		0,      // Image stream flags, e.g. near mode
		2,      // Number of frames to buffer
		NULL,   // Event handle
		&rgbStream);
	sensor->NuiSkeletonTrackingEnable(NULL, 0); // NUI_SKELETON_TRACKING_FLAG_ENABLE_SEATED_SUPPORT for only upper body
	return sensor;
}

void getDepthData(GLubyte* dest) {
	float* fdest = (float*)dest;
	long* depth2rgb = (long*)depthToRgbMap;
	NUI_IMAGE_FRAME imageFrame;
	NUI_LOCKED_RECT LockedRect;
	if (sensor->NuiImageStreamGetNextFrame(depthStream, 0, &imageFrame) < 0) return;
	INuiFrameTexture* texture = imageFrame.pFrameTexture;
	texture->LockRect(0, &LockedRect, NULL, 0);
	if (LockedRect.Pitch != 0) {
		const USHORT* curr = (const USHORT*)LockedRect.pBits;
		for (int j = 0; j < height; ++j) {
			for (int i = 0; i < width; ++i) {
				// Get depth of pixel in millimeters
				USHORT depth = NuiDepthPixelToDepth(*curr++);
				// Store coordinates of the point corresponding to this pixel
				Vector4 pos = NuiTransformDepthImageToSkeleton(i, j, depth << 3, NUI_IMAGE_RESOLUTION_640x480);
				*fdest++ = pos.x / pos.w;
				*fdest++ = pos.y / pos.w;
				*fdest++ = pos.z / pos.w;
				// Store the index into the color array corresponding to this pixel
				NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution(
					NUI_IMAGE_RESOLUTION_640x480, NUI_IMAGE_RESOLUTION_640x480, NULL,
					i, j, depth << 3, depth2rgb, depth2rgb + 1);
				depth2rgb += 2;
			}
		}
	}
	texture->UnlockRect(0);
	sensor->NuiImageStreamReleaseFrame(depthStream, &imageFrame);
}

void getRgbData(GLubyte* dest) {
	float* fdest = (float*)dest;
	long* depth2rgb = (long*)depthToRgbMap;
	NUI_IMAGE_FRAME imageFrame;
	NUI_LOCKED_RECT LockedRect;
	if (sensor->NuiImageStreamGetNextFrame(rgbStream, 0, &imageFrame) < 0) return;
	INuiFrameTexture* texture = imageFrame.pFrameTexture;
	texture->LockRect(0, &LockedRect, NULL, 0);
	if (LockedRect.Pitch != 0) {
		const BYTE* start = (const BYTE*)LockedRect.pBits;
		for (int j = 0; j < height; ++j) {
			for (int i = 0; i < width; ++i) {
				// Determine rgb color for each depth pixel
				long x = *depth2rgb++;
				long y = *depth2rgb++;
				// If out of bounds, then don't color it at all
				if (x < 0 || y < 0 || x > width || y > height) {
					for (int n = 0; n < 3; ++n) *(fdest++) = 0.0f;
				}
				else {
					const BYTE* curr = start + (x + width*y) * 4;
					for (int n = 0; n < 3; ++n) *(fdest++) = curr[2 - n] / 255.0f;
				}

			}
		}
	}
	texture->UnlockRect(0);
	sensor->NuiImageStreamReleaseFrame(rgbStream, &imageFrame);
}

void getSkeletalData() {
	NUI_SKELETON_FRAME skeletonFrame = { 0 };
	if (sensor->NuiSkeletonGetNextFrame(0, &skeletonFrame) >= 0) {
		sensor->NuiTransformSmooth(&skeletonFrame, NULL);
		// Loop over all sensed skeletons
		for (int z = 0; z < NUI_SKELETON_COUNT; ++z) {
			const NUI_SKELETON_DATA& skeleton = skeletonFrame.SkeletonData[z];
			// Check the state of the skeleton
			if (skeleton.eTrackingState == NUI_SKELETON_TRACKED) {
				// Copy the joint positions into our array
				for (int i = 0; i < NUI_SKELETON_POSITION_COUNT; ++i) {
					skeletonPosition[i] = skeleton.SkeletonPositions[i];
					if (skeleton.eSkeletonPositionTrackingState[i] == NUI_SKELETON_POSITION_NOT_TRACKED) {
						skeletonPosition[i].w = 0;
					}
				}
				return; // Only take the data for one skeleton
			}
		}
	}
}
void getKinectData() {
	const int dataSize = width*height * 3 * 4;
	GLubyte* ptr;
	glBindBuffer(GL_ARRAY_BUFFER, vboId);
	ptr = (GLubyte*)glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
	if (ptr) {
		getDepthData(ptr);
	}
	glUnmapBuffer(GL_ARRAY_BUFFER);
	glBindBuffer(GL_ARRAY_BUFFER, cboId);
	ptr = (GLubyte*)glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
	if (ptr) {
		getRgbData(ptr);
	}
	glUnmapBuffer(GL_ARRAY_BUFFER);
	getSkeletalData();
}

void rotateCamera() {
	static double angle = 0.;
	static double radius = 3.;
	double x = radius*sin(angle);
	double z = radius*(1 - cos(angle)) - radius / 2;
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(x, 0, z, 0, 0, radius / 2, 0, 1, 0);
	//angle = 0.2;
	mov += 1.5;
}

void drawKinectData() {
	getKinectData();
	rotateCamera();

	glMatrixMode(GL_MODELVIEW);
	glMatrixMode(GL_PROJECTION);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_COLOR_ARRAY);

	glBindBuffer(GL_ARRAY_BUFFER, vboId);
	glVertexPointer(3, GL_FLOAT, 0, NULL);

	glBindBuffer(GL_ARRAY_BUFFER, cboId);
	glColorPointer(3, GL_FLOAT, 0, NULL);

	glPointSize(1.f);
	glDrawArrays(GL_POINTS, 0, width*height);

	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);

	// Draw some arms
	const Vector4& lh = skeletonPosition[NUI_SKELETON_POSITION_HAND_LEFT];
	const Vector4& le = skeletonPosition[NUI_SKELETON_POSITION_ELBOW_LEFT];
	const Vector4& ls = skeletonPosition[NUI_SKELETON_POSITION_SHOULDER_LEFT];
	const Vector4& rh = skeletonPosition[NUI_SKELETON_POSITION_HAND_RIGHT];
	const Vector4& re = skeletonPosition[NUI_SKELETON_POSITION_ELBOW_RIGHT];
	const Vector4& rs = skeletonPosition[NUI_SKELETON_POSITION_SHOULDER_RIGHT];
	
	glBegin(GL_LINES);
	glColor3f(1.f, 0.f, 0.f);
	if (lh.w > 0 && le.w > 0 && ls.w > 0) {
		glVertex3f(lh.x, lh.y, lh.z);
		glVertex3f(le.x, le.y, le.z);
		glVertex3f(le.x, le.y, le.z);
		glVertex3f(ls.x, ls.y, ls.z);
	}
	lhX = lh.x; lhY = lh.y; lhZ = lh.z;
	if (rh.w > 0 && re.w > 0 && rs.w > 0) {
		glVertex3f(rh.x, rh.y, rh.z);
		glVertex3f(re.x, re.y, re.z);
		glVertex3f(re.x, re.y, re.z);
		glVertex3f(rs.x, rs.y, rs.z);
	}
	glEnd();
	int mapX = (int)(lhX * 10);
	int mapY = (int)(lhY * 10);
	int mapZ = (int)(lhZ * 10);

	if (timer % 150 == 0) {
		x = (rand() % 1000);
		y = (rand() % 1000);
		z = (rand() % 1000);
		X = (double)x / 1000 - 0.5;
		Y = (double)y / 1000 - 0.5;
		Z = (double)z / 1000 - 0.5;
		randX = (int)(X * 10);
		randY = (int)(Y * 10);
		randZ = (int)(Z * 10);
	}
	drawHammer();
	
	if (randX >= -mapX - 0.5 && randX < -mapX + 0.5) {
		if (randY >= mapY - 0.5 && randY < mapY + 0.5) {
			x = (rand() % 1000);
			y = (rand() % 1000);
			z = (rand() % 1000);
			X = (double)x / 1000 - 0.5;
			Y = (double)y / 1000 - 0.5;
			Z = (double)z / 1000 - 0.5;
			randX = (int)(X * 10);
			randY = (int)(Y * 10);
			randZ = (int)(Z * 10);
			//puts("Hit!1");
			DrawHit();
			delay++;
			hit++;
		}
	}
	if (delay != 0) {
		if (delay != 4) {
			delay++;
			DrawHit();
		}
		else delay = 0;
	}
	hei += change;
	if (hei > 15)
		change = -1;
	else if (hei < 8)
		change = 1;

	glTranslatef(X, Y, Z);
	createDigda(X, Y, Z, 0.08, (float)hei/100);

	timer++;
	printf(" *%d* \n", timer);
	if (timer > 500) {
		printf("*************CATCHING DIGDA!***********\n\n");
		printf("      The number of hit : %d\n",hit);
		printf("    X The score per 1 Digda : %d\n", 16);
		printf("----------------------------------\n\n");
		printf("***************************************\n");
		printf("                    Total : %d\n", hit*16);
		if (hit > 2)
			printf("   Great job! You're a digda master!\n");
		else
			printf("It's okay. You will get better next time!\n");
		exit(0);
	}
	glPopMatrix();
}

int main(int argc, char* argv[]) {
	if (!init(argc, argv)) return 1;
	if (!initKinect()) return 1;
	srand(time(NULL));

	// OpenGL setup
	glClearColor(0, 0, 0, 0);
	glClearDepth(1.0f);

	//background music setup
	char music[] = "we_go_together.wav";
	wchar_t music1[30];
	mbstowcs(music1, music, strlen(music) + 1);
	LPCWSTR nameMu = music1;
	PlaySound(nameMu, NULL, SND_ASYNC | SND_FILENAME | SND_LOOP);
	
	// Set up array buffers
	const int dataSize = width*height * 3 * 4;
	glGenBuffers(1, &vboId);
	glBindBuffer(GL_ARRAY_BUFFER, vboId);
	glBufferData(GL_ARRAY_BUFFER, dataSize, 0, GL_DYNAMIC_DRAW);
	glGenBuffers(1, &cboId);
	glBindBuffer(GL_ARRAY_BUFFER, cboId);
	glBufferData(GL_ARRAY_BUFFER, dataSize, 0, GL_DYNAMIC_DRAW);

	// Camera setup
	glViewport(0, 0, width, height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45, width / (GLdouble)height, 0.1, 1000);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(0, 0, 0, 0, 0, 1, 0, 1, 0);
	glutDisplayFunc(drawKinectData);
	// Main loop
	execute();
	return 0;
}
void createBody(GLfloat centerx, GLfloat centery, GLfloat centerz, GLfloat radius, GLfloat h, GLfloat color[3]) {
	GLfloat x, z, angle;
	//glTranslatef(centerx, centery, centerz);
	glPushMatrix();

	glBegin(GL_TRIANGLE_FAN);           //원기둥의 밑면
	glNormal3f(0.0f, 1.0f, 0.0f);
	glVertex3f(centerx, centery, centerz);
	glColor3ub(80, 80, 80);
	for (angle = (3.0f*GL_PI); angle > 0.0f; angle -= (GL_PI / 12.0f))
	{
		x = centerx + (radius*1.5)*sin(angle);
		z = centerz + (radius*1.5)*cos(angle);
		glNormal3f(0.0f, 1.0f, 0.0f);
		glVertex3f(x,centery,z);
	}
	glEnd();

	glBegin(GL_QUAD_STRIP);          //원기둥의 옆면
	glColor3ub(color[0], color[1], color[2]);
	for (angle = 0.0f; angle < (3.0f*GL_PI); angle += (GL_PI / 12.0f))
	{
		x = centerx + radius*sin(angle);
		z = centerz + radius*cos(angle);
		glNormal3f(sin(angle),0.0f, cos(angle));
		glVertex3f(x,centery,z);
		glVertex3f(x,centery+h,z);
	}
	glEnd();
	glPopMatrix();
}
void createDigda(GLfloat centerx, GLfloat centery, GLfloat centerz, GLfloat radius, GLfloat h)
{

	glPushMatrix();
	 createBody(0, 0, 0, radius, h, Body);

	glPushMatrix();
	glTranslatef(0, h, 0);
	createHalfSphere(centerx, centery, centerz, radius, Body);//머리

	glPushMatrix();
	glTranslatef(-radius*0.5, 0, 0);
	DrawEllipse(radius*0.15, radius*0.25, eye);//눈1

	glTranslatef(radius, 0, 0);
	DrawEllipse(radius*0.15, radius*0.25, eye);//눈2

	glTranslatef(-radius*0.5, -radius*0.4, 0);
	DrawEllipse(radius*0.25, radius*0.15, nose);//코
	  glPopMatrix();
	glPopMatrix();


	glPushMatrix();  //돌
	//glTranslatef(centerx-radius,centery-(radius * 2),centerz-radius);
	glTranslatef(-radius,0,0);
	createHalfSphere(centerx, centery, centerz, radius*(0.4), stone);

	glTranslatef(radius*2.25, 0, 0);
	createHalfSphere(centerx, centery, centerz, radius*0.25, stone);

	//glTranslatef(0, radius*1.5, -radius*0.9);
	glTranslatef(-radius*1.2, 0, 0);
	createHalfSphere(centerx, centery, centerz, radius*0.25, stone);

	glTranslatef(radius*0.25, 0, radius*0.25);
	createHalfSphere(centerx, centery, centerz, radius*0.15, stone);
	glPopMatrix();
	glPopMatrix();
	//glPopMatrix();
}
void createHalfSphere(GLfloat x, GLfloat y, GLfloat z, GLfloat radius, GLfloat color[3])
{
	GLfloat angley;  //y축 값을 구하기 위한 각도
	GLfloat nexty;  //다음 y축 값을 구하기 위한 각도
	GLfloat anglex;  //x, y축 값을 구하기 위한 각도

	glColor3ub(color[0], color[1], color[2]);  //반구의 색 지정
	glBegin(GL_QUAD_STRIP);
	for (angley = 0.0f; angley <= (0.5f*GL_PI); angley += ((0.5f*GL_PI) / 10.0f))  //반구만 그려야 하므로 0.5곱함
	{
		y = radius*sin(angley);     //y축 값 계산
		nexty = angley + ((0.5f*GL_PI) / 10.0f);  //다음 angley값 저장
		for (anglex = 0.0f; anglex < (3.0f*GL_PI); anglex += (GL_PI / 10.0f))
		{
			x = radius*cos(angley)*sin(anglex);
			z = radius*cos(angley)*cos(anglex);
			glNormal3f(-cos(angley)*sin(anglex), -sin(angley), -cos(angley)*cos(anglex)); //반구의 안쪽으로 normal 벡터 생성
			glVertex3f(x, y, z);

			x = radius*cos(nexty)*sin(anglex);
			z = radius*cos(nexty)*cos(anglex);
			glNormal3f(-cos(nexty)*sin(anglex), -sin(nexty), -cos(nexty)*cos(anglex));
			glVertex3f(x, radius*sin(nexty), z);
		}
	}
	glEnd();
}
void DrawEllipse(float radiusX, float radiusY, GLfloat color[3])
{
	int i;

	glBegin(GL_POLYGON);
	glColor3ub(color[0], color[1], color[2]);

	for (i = 0; i<360; i++)
	{
		float rad = i*DEG2RAD;
		glVertex2f(cos(rad)*radiusX, sin(rad)*radiusY);
	}

	glEnd();
}
void drawStick(float size, int times) {
	for (int i = 0; i < times; i++) {
		glTranslatef(0, -size, 0);
		glutSolidCube(size);
	}
}
void drawBar(float size, int times) {
	for (int i = 0; i < times; i++) {
		glTranslatef(size, 0, 0);
		glutSolidCube(size);
	}
}
void DrawHit() {
	glColor3f(1, 1, 0);
	float size = 0.02;

	glPushMatrix();
	drawStick(size, 6); // H1
	glPopMatrix();

	glPushMatrix();
	glTranslatef(0, -size * 3, 0); //H3
	drawBar(size, 3);
	glPopMatrix();

	glTranslatef(size*3, 0, 0); //H2
	glPushMatrix();
	drawStick(size, 6);
	glPopMatrix();

	glTranslatef(size*2, 0, 0);
	glPushMatrix();
	drawStick(size, 1); //i1
	glTranslatef(0, -size, 0);
	drawStick(size, 4); //i2
	glPopMatrix();

	glTranslatef(size*2, 0, 0);
	glPushMatrix();
	drawStick(size, 6); //t1
	glPopMatrix();

	glPushMatrix();
	glTranslatef(-size * 2, -size, 0);
	drawBar(size, 3); //t2
	glPopMatrix();

	glTranslatef(size*3, 0, 0);
	glPushMatrix();
	drawStick(size, 4); //!1
	glTranslatef(0, -size, 0);
	drawStick(size, 1); //!2
	glPopMatrix();
}
void tgdrawHammer() {
	glPushMatrix();      // draw hammer
	glRotatef(lhY * 100, 0, 0, 1);
	glTranslatef(-0.05, -0.01, 0);
	glColor3f(1, 0.9, 0.2);
	glRectf(lhX + 0.04, lhY / 2 + 0.11, lhX - 0.09, lhY / 2 + 0.15);//handle
	glColor3f(1.0, 0.2, 0.4);
	glRectf(lhX - 0.01, lhY / 2 - 0.06, lhX + 0.03, lhY / 2 + 0.11);
	glColor3f(0.f, 0.1, 1.f);
	glRectf(lhX - 0.07, lhY / 2 + 0.11, lhX - 0.09, lhY / 2 + 0.15);//hammer
	glPopMatrix();
	glPushMatrix();
}
void drawHammer() {
	glPushMatrix();      // draw hammer
	glRotatef(lhY * 100, 0, 0, 1);
	glTranslatef(-0.05, -0.01, 0);
	glColor3f(1.0, 0.2, 0.4);
	//glColor3f(0, 1, 0);
	glRectf(lhX - 0.015, lhY / 2 - 0.06, lhX + 0.015, lhY / 2 + 0.18);//handle
	glColor3f(0.f, 0.1, 1.f);
	//glColor3f(0, 0, 1);
	glRectf(lhX - 0.07, lhY / 2 + 0.07, lhX - 0.09, lhY / 2 + 0.15);//b
	glRectf(lhX + 0.07, lhY / 2 + 0.07, lhX + 0.09, lhY / 2 + 0.15);//b
	glColor3f(1, 0.9, 0.2);
	//glColor3f(1, 0, 0);
	glRectf(lhX - 0.07, lhY / 2 + 0.07, lhX + 0.07, lhY / 2 + 0.15);//y
	glPopMatrix();
	glPushMatrix();
}