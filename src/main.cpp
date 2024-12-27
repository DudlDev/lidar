#include <stdio.h>
#include <stdlib.h>

#include "glad/glad.h"		// link both libraries (gl 330 core)
#include "GLFW/glfw3.h"		// -------------------

#include "rplidar.h"		// the official sdk from github

/*
Node:
	sl_u16   angle_z_q14;
	sl_u32   dist_mm_q2;
	sl_u8    quality;
	sl_u8    flag;
*/

// S: change zoom, M: change rendering mode, Space/Backspace: take a snapshot (save data) and pause / resume

#define ERRORIFNULL(var, str) if(!var) { printf(str); return 0; }

using namespace rp::standalone::rplidar;

// size of the glfw window
size_t windowWidth = 1024;
size_t windowHeight = 1024;

bool snapshot = 0;


char* readFile(const char* path)
{
	FILE* f;
	fopen_s(&f, path, "r");
	if (!f) return 0;

	size_t len = 0;
	while (fgetc(f) != EOF) len++;
	fseek(f, 0, SEEK_SET);

	char* buf = (char*)malloc(len + 1);
	if (!buf) return 0;

	fread(buf, 1, len, f);
	buf[len] = 0;

	return buf;
}

GLuint createShader(const char* vsPath, const char* fsPath)
{
	char* vertexSource = readFile(vsPath);
	char* fragmentSource = readFile(fsPath);

	GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
	GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);

	glShaderSource(vertexShader, 1, &vertexSource, 0);
	glShaderSource(fragmentShader, 1, &fragmentSource, 0);

	glCompileShader(vertexShader);
	glCompileShader(fragmentShader);

	free(vertexSource);
	free(fragmentSource);

	int success;
	char infoLog[512];
	glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
	if (!success)
	{
		glGetShaderInfoLog(vertexShader, 512, NULL, infoLog);
		printf("%s", infoLog);
	}
	glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);
	if (!success)
	{
		glGetShaderInfoLog(fragmentShader, 512, NULL, infoLog);
		printf("%s", infoLog);
	}

	GLuint shader = glCreateProgram();
	glAttachShader(shader, vertexShader);
	glAttachShader(shader, fragmentShader);
	glLinkProgram(shader);

	glGetProgramiv(shader, GL_LINK_STATUS, &success);
	if (!success) 
	{
		glGetProgramInfoLog(shader, 512, NULL, infoLog);
		printf("%s", infoLog);
	}

	glUseProgram(shader);

	glDeleteShader(vertexShader);
	glDeleteShader(fragmentShader);

	return shader;
}

GLFWwindow* initGL()
{
	ERRORIFNULL(glfwInit(), "GLFW couldn't init.");

	GLFWwindow* window = glfwCreateWindow(windowWidth, windowHeight, "Lidar", 0, 0);
	ERRORIFNULL(window, "Window couldn't open.");

	glfwMakeContextCurrent(window);

	ERRORIFNULL(gladLoadGLLoader((GLADloadproc)glfwGetProcAddress), "GLAD couldn't get the proc address.");

	glViewport(0, 0, windowWidth, windowHeight);

	return window;
}

RPlidarDriver* initRPlidar()
{
	// create the driver instance
	RPlidarDriver* driver = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);

	// this may differ depending on the os, this is for windows, check the com port in the device manager to find the right one
	driver->connect("\\\\.\\com5", 115200);

	driver->startMotor();
	driver->startScan(0, 1);

	return driver;
}

// this was used for debugging
void printBits(uint32_t num, size_t size)
{
	printf("0b");

	for (int i = size - 1; i >= 0; i--)
	{
		printf("%d", (num & (0x1u << i)) >> i);
	}

	printf("\n");
}


// a struct to convert each scan, is only used for CSV, not for the rendering
struct Measurement
{
	float distance;
	float angle;

	sl_u8 flag;
	sl_u8 quality;

	Measurement(sl_lidar_response_measurement_node_hq_t from)
		: distance(from.dist_mm_q2 / 4.0f * 0.001f),
		angle(from.angle_z_q14 * 90.f / (1 << 14)),
		flag(from.flag),
		quality(from.quality)
	{}

	rplidar_response_measurement_node_hq_t toRP()
	{
		return { (sl_u16)(angle * (1 << 14) / 90.0f), (sl_u32)(distance * 4.0f * 1000.0f), quality, flag };
	}
};

int rpDataToCSV(const char* path, rplidar_response_measurement_node_hq_t* buffer, size_t count)
{
	FILE* f;
	fopen_s(&f, path, "w");

	if (!f) return 0;

	fprintf(f, "Angle;Distance\n");

	for (int i = 0; i < count; i++)
	{
		Measurement m(buffer[i]);

		fprintf(f, "%.4f;%.4f\n", m.angle, m.distance);
	}

	fclose(f);

	return 1;

}

int main()
{
	GLFWwindow* window = initGL();

	GLuint VBO, VAO;
	{
		glGenVertexArrays(1, &VAO);
		glGenBuffers(1, &VBO);

		glBindVertexArray(VAO);
		glBindBuffer(GL_ARRAY_BUFFER, VBO);

		// can store way too much data, change later
		glBufferData(GL_ARRAY_BUFFER, sizeof(sl_u64) * 8192, 0, GL_STREAM_DRAW);

		// each "vertex" is a single scan, which is send to the gpu as a 2d u32 vector
		// important to use "glVertexAttribIPointer" (with an I!!!) so the data will not be interpolated
		glVertexAttribIPointer(0, 2, GL_UNSIGNED_INT, sizeof(sl_u64), (void*)(0));
		glEnableVertexAttribArray(0);
	}

	GLuint shader = createShader("res\\shader\\default.vert", "res\\shader\\default.frag");
	GLint uniformColor = glGetUniformLocation(shader, "color");
	GLint uniformPolar = glGetUniformLocation(shader, "polar");
	GLint uniformScale = glGetUniformLocation(shader, "scale");

	RPlidarDriver* driver = initRPlidar();

	// these are way too many samples, depending on scan frequency (~7Hz for the A1), only about 1070 samples will be aquired
	rplidar_response_measurement_node_hq_t* nodes = new rplidar_response_measurement_node_hq_t[8192];
	size_t count = 8192;

	double lastTime = 0.0;
	double deltaTime = 0.0;

	// for input
	int mode = 0;
	int modeDown = 0;

	float scale = 9.0f;
	int scaleDown = 0;

	glPointSize(3.0f);

	while (!glfwWindowShouldClose(window))
	{
		deltaTime = glfwGetTime() - lastTime;
		lastTime = glfwGetTime();


		glfwPollEvents();

		if (glfwGetKey(window, GLFW_KEY_SPACE))
		{
			if (snapshot == 0)
			{
				// change this to a desired path
				rpDataToCSV("C:\\measurements\\raw.csv", nodes, count);

				snapshot = 1;
			}
		}
		if (glfwGetKey(window, GLFW_KEY_BACKSPACE)) snapshot = 0;

		if (glfwGetKey(window, GLFW_KEY_M))
		{
			if (modeDown == 0)
			{
				mode++;
				if (mode > 3) mode = 0;

				modeDown = 1;
			}
		}
		else
			modeDown = 0;

		if (glfwGetKey(window, GLFW_KEY_S))
		{
			if (scaleDown == 0)
			{
				scale += 1.0f;
				if (scale > 12.5f) scale = 1.0f;

				scaleDown = 1;
			}
		}
		else
			scaleDown = 0;


		u_result result = 0;

		double dtScan = 0.0;
		if (!snapshot)
		{
			double t0Scan = glfwGetTime();

			result = driver->grabScanDataHq(nodes, count);

			dtScan = glfwGetTime() - t0Scan;
		}

		double dtBuffer = 0.0;
		if (IS_OK(result) && !snapshot)
		{
			double t0Buffer = glfwGetTime();

			glBindBuffer(GL_ARRAY_BUFFER, VBO);

			// the aquired scans are converted on the gpu, so no conversion has to be done on the cpu (this is way more efficient9
			glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(sl_u64) * count, nodes);

			dtBuffer = glfwGetTime() - t0Buffer;
		}


		double t0Draw = glfwGetTime();

		glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT);

		glUseProgram(shader);
		glBindVertexArray(VAO);
		glBindBuffer(GL_ARRAY_BUFFER, VBO);

		glUniform1f(uniformScale, scale);
		glUniform3f(uniformColor, mode > 1 ? 1.0f : 0.0f, 1.0f, 1.0f);
		glUniform1i(uniformPolar, mode % 2 == 0);
	
		glDrawArrays(GL_POINTS, 0, count);

		glfwSwapBuffers(window);

		double dtDraw = glfwGetTime() - t0Draw;


		printf("\x1b[HSamples: %d						\n", (int)count);
		printf("Scan Frequenzy: %fHz					\n", 1.0 / dtScan);
		printf("Samples per Second: %d					\n", (int)(count / dtScan));
		printf("Buffer Time: %fs						\n", dtBuffer);
		printf("Draw Time: %fs							\n", dtDraw);
		printf("FPS: %fHz								\n", 1.0 / deltaTime);
		printf("Distance: %f							\n", Measurement(nodes[0]).distance);
		printf("Mode: %d								\n", mode);
		printf("Scale: %dm                              \n", 13 - (int)scale);
	}

	glfwTerminate();
}