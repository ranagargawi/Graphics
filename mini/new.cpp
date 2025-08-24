#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GLFW/glfw3.h>

struct Vertex {
    float x, y, z;
};

std::vector<Vertex> vertices;
std::vector<unsigned int> indices;
std::vector<std::tuple<float, float, float>> cameraPath;

float camX = 0, camY = -100, camZ = 100;
float camSpeed = 2.0f;

void loadDEM(const std::string& filename) {
    cv::Mat img = cv::imread(filename, cv::IMREAD_GRAYSCALE);
    if (img.empty()) {
        std::cerr << "Failed to load image: " << filename << std::endl;
        exit(1);
    }

    int rows = img.rows;
    int cols = img.cols;

    for (int i = 0; i < rows; ++i)
        for (int j = 0; j < cols; ++j)
            vertices.push_back({(float)j, (float)i, (float)img.at<uchar>(i, j) / 5.0f});

    for (int i = 0; i < rows - 1; ++i) {
        for (int j = 0; j < cols - 1; ++j) {
            int idx = i * cols + j;
            indices.push_back(idx);
            indices.push_back(idx + 1);
            indices.push_back(idx + cols);

            indices.push_back(idx + 1);
            indices.push_back(idx + cols + 1);
            indices.push_back(idx + cols);
        }
    }
}

void drawMesh() {
    glBegin(GL_TRIANGLES);
    for (size_t i = 0; i < indices.size(); i += 3) {
        Vertex& v1 = vertices[indices[i]];
        Vertex& v2 = vertices[indices[i + 1]];
        Vertex& v3 = vertices[indices[i + 2]];

        glVertex3f(v1.x, v1.y, v1.z);
        glVertex3f(v2.x, v2.y, v2.z);
        glVertex3f(v3.x, v3.y, v3.z);
    }
    glEnd();
}

void drawCameraPath() {
    glColor3f(1, 0, 0);
    glBegin(GL_LINE_STRIP);
    for (auto& [x, y, z] : cameraPath)
        glVertex3f(x, y, z);
    glEnd();
}

void processInput(GLFWwindow* window) {
    bool moved = false;
    if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS) { camY += camSpeed; moved = true; }
    if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS) { camY -= camSpeed; moved = true; }
    if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS) { camX -= camSpeed; moved = true; }
    if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS) { camX += camSpeed; moved = true; }
    if (glfwGetKey(window, GLFW_KEY_PAGE_UP) == GLFW_PRESS) { camZ += camSpeed; moved = true; }
    if (glfwGetKey(window, GLFW_KEY_PAGE_DOWN) == GLFW_PRESS) { camZ -= camSpeed; moved = true; }

    if (moved)
        cameraPath.emplace_back(camX, camY, camZ);
}

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: ./terrain_viewer <image_path>" << std::endl;
        return 1;
    }

    loadDEM(argv[1]);

    if (!glfwInit()) return -1;
    GLFWwindow* window = glfwCreateWindow(800, 600, "Terrain Navigation", nullptr, nullptr);
    if (!window) return -1;
    glfwMakeContextCurrent(window);
    glEnable(GL_DEPTH_TEST);

    while (!glfwWindowShouldClose(window)) {
        processInput(window);

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glLoadIdentity();
        gluLookAt(camX, camY, camZ, 50, 50, 0, 0, 0, 1);

        drawMesh();
        drawCameraPath();

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}
