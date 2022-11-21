#include <stdio.h>
#include <fstream>
#include <iostream>
#include <cstdlib>
#include <math.h>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

//#define USE_EULERS
#define LSM303DHLC
#define READ_SERIAL
#include "serial.h"

static char msg[256];
static float roll, pitch, yaw;
static float w, x, y, z;
static GLfloat matrix[16];

Serial serial;

static GLFWwindow *window;

void quat_to_matrix(float w, float x, float y, float z)
{
	float x2 = x * x; float y2 = y * y; float z2 = z * z;
	float wx = w * x; float wy = w * y; float wz = w * z;
	float xy = x * y; float xz = x * z; float yz = y * z;

	matrix[0] = 1 - 2 * (y2 + z2); matrix[4] = 2 * (xy - wz);     matrix[8] = 2 * (xz + wy);      matrix[12] = 0;
	matrix[1] = 2 * (xy + wz);     matrix[5] = 1 - 2 * (x2 + z2); matrix[9] = 2 * (yz - wx);      matrix[13] = 0;
	matrix[2] = 2 * (xz - wy);     matrix[6] = 2 * (yz + wx);     matrix[10] = 1 - 2 * (x2 + y2); matrix[14] = 0;
	matrix[3] = 0;                 matrix[7] = 0;                 matrix[11] = 0;                 matrix[15] = 1;
}

void gl_perspective(GLdouble fovY, GLdouble aspect, GLdouble zNear,
		   GLdouble zFar)
{
	GLdouble fW, fH;
	fH = tan(fovY / 360 * M_PI) * zNear;
	fW = fH * aspect;
	glFrustum(-fW, fW, -fH, fH, zNear, zFar);
}

void resize_callback(GLFWwindow *window, int width, int height)
{
	glViewport(0, 0, width, height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gl_perspective(45, width / height, 0.1, 100.0);
}

void key_callback(GLFWwindow *window, int key, int scancode, int action,
		  int mods)
{
	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
		glfwSetWindowShouldClose(window, GLFW_TRUE);
}

bool create_window()
{
	window = glfwCreateWindow(640, 480, "IMU orientation visualizer", NULL, NULL);
	if (!window) {
		glfwTerminate();
		return false;
	}

	glfwMakeContextCurrent(window);
	glfwSetKeyCallback(window, key_callback);
	glfwSetFramebufferSizeCallback(window, resize_callback);
	return true;
}

void draw()
{
	glMatrixMode(GL_MODELVIEW);

#ifndef USE_EULERS
	glPushMatrix();
	glTranslatef(0.0f, 0.0f, -7.0f);
	glMultMatrixf(matrix);
#else
	glLoadIdentity();
	glTranslatef(0.0f, 0.0f, -7.0f);
	glRotatef(roll, 1, 0, 0);
	glRotatef(-pitch, 0, 0, 1);
	glRotatef(yaw, 0, 1, 0);
#endif

	glBegin(GL_QUADS);
	// Top face (y = 1.0f)
	// Define vertices in counter-clockwise (CCW) order with normal pointing out
	glColor3f(0.0f, 1.0f, 0.0f); // Green
	glVertex3f(1.5f, 0.25f, -1.0f);
	glVertex3f(-1.5f, 0.25f, -1.0f);
	glVertex3f(-1.5f, 0.25f, 1.0f);
	glVertex3f(1.5f, 0.25f, 1.0f);

	// Bottom face (y = -1.0f)
	glColor3f(1.0f, 0.5f, 0.0f); // Orange
	glVertex3f(1.5f, -0.25f, 1.0f);
	glVertex3f(-1.5f, -0.25f, 1.0f);
	glVertex3f(-1.5f, -0.25f, -1.0f);
	glVertex3f(1.5f, -0.25f, -1.0f);

	// Front face  (z = 1.0f)
	glColor3f(1.0f, 0.0f, 0.0f); // Red
	glVertex3f(1.5f, 0.25f, 1.0f);
	glVertex3f(-1.5f, 0.25f, 1.0f);
	glVertex3f(-1.5f, -0.25f, 1.0f);
	glVertex3f(1.5f, -0.25f, 1.0f);

	// Back face (z = -1.0f)
	glColor3f(1.0f, 1.0f, 0.0f); // Yellow
	glVertex3f(1.5f, -0.25f, -1.0f);
	glVertex3f(-1.5f, -0.25f, -1.0f);
	glVertex3f(-1.5f, 0.25f, -1.0f);
	glVertex3f(1.5f, 0.25f, -1.0f);

	// Left face (x = -1.0f)
	glColor3f(0.0f, 0.0f, 1.0f); // Blue
	glVertex3f(-1.5f, 0.25f, 1.0f);
	glVertex3f(-1.5f, 0.25f, -1.0f);
	glVertex3f(-1.5f, -0.25f, -1.0f);
	glVertex3f(-1.5f, -0.25f, 1.0f);

	// Right face (x = 1.0f)
	glColor3f(1.0f, 0.0f, 1.0f); // Magenta
	glVertex3f(1.5f, 0.25f, -1.0f);
	glVertex3f(1.5f, 0.25f, 1.0f);
	glVertex3f(1.5f, -0.25f, 1.0f);
	glVertex3f(1.5f, -0.25f, -1.0f);
	glEnd();

#ifndef USE_EULERS
	glPopMatrix();
#endif
}

void get_rpy()
{
	if (!serial.readLine(msg, sizeof(msg) - 1))
		return;

	char *start = msg;
	char *ptr;
	char *end = msg + sizeof(msg);

	if (!(ptr = strchr(start, ' '))) {
		return;
	} else {
		*ptr = '\0';
		roll = strtof(start, NULL);
	}

	start = ptr + 1;
	if (!(ptr = strchr(start, ' '))) {
		return;
	} else {
		*ptr = '\0';
		pitch = strtof(start, NULL);
	}

	yaw = strtof(ptr + 1, NULL);
	printf("roll: %f pitch: %f yaw: %f\n", roll, pitch, yaw);
}

void get_orientation()
{
	if (!serial.readLine(msg, sizeof(msg) - 1))
		return;

	char *start = msg;
	char *ptr;
	char *end = msg + sizeof(msg);

	if (!(ptr = strchr(start, ' '))) {
		return;
	} else {
		*ptr = '\0';
		w = strtof(start, NULL);
	}

	start = ptr + 1;
	if (!(ptr = strchr(start, ' '))) {
		return;
	} else {
		*ptr = '\0';
		x = strtof(start, NULL);
	}

	start = ptr + 1;
	if (!(ptr = strchr(start, ' '))) {
		return;
	} else {
		*ptr = '\0';
		y = strtof(start, NULL);
	}

	z = strtof(ptr + 1, NULL);
	printf("w: %f x: %f y: %f z: %f\n", w, x, y, z);
	
#ifdef LSM303DHLC  
	quat_to_matrix(w, x, z, -y); 
#else 
	quat_to_matrix(w, -x, z, -y);
#endif
}

bool init()
{
	if (!glfwInit()) {
		printf("Window initialization failed\n");
		return false;
	}

	if (!create_window()) {
		printf("Window creation failed\n");
		return false;
	}

	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
		printf("Failed to load GLAD\n");
		return false;
	}

	resize_callback(window, 640, 480);
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f); // Set background color to black and opaque
	glClearDepth(1.0f); // Set background depth to farthest
	glEnable(GL_DEPTH_TEST); // Enable depth testing for z-culling
	glDepthFunc(GL_LEQUAL); // Set the type of depth-test
	glShadeModel(GL_SMOOTH); // Enable smooth shading
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST); // Nice perspective corrections

#ifdef READ_SERIAL
	serial.openSerial();
#endif
	return true;
}

int main()
{
	if (!init())
		return -1;

	while (!glfwWindowShouldClose(window)) {
#if defined(READ_SERIAL) && defined(USE_EULERS)
		get_rpy();
#elif defined(READ_SERIAL)
		get_orientation();
#endif
		glfwPollEvents();

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		draw();

		glfwSwapBuffers(window);
	}

#ifdef READ_SERIAL
	serial.closeSerial();
#endif
	glfwDestroyWindow(window);
	glfwTerminate();
	return 0;
}
