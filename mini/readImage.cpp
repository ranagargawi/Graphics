#include <opencv2/opencv.hpp>
#include <GL/glut.h>
#include <vector>
#include <iostream>
#include <cmath>
#include <cfloat>

using namespace std;
using namespace cv;

float camX = 400, camY = 300, camZ = 800;
float yaw = 0.0f;
float pitch = -30.0f;
float moveSpeed = 10.0f;
float turnSpeed = 5.0f;

struct MyVec3f {
    float x, y, z;
    MyVec3f() : x(0), y(0), z(0) {}
    MyVec3f(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    MyVec3f operator-(const MyVec3f &v) const {
        return MyVec3f(x - v.x, y - v.y, z - v.z);
    }

    MyVec3f operator+(const MyVec3f &v) const {
        return MyVec3f(x + v.x, y + v.y, z + v.z);
    }

    MyVec3f cross(const MyVec3f &v) const {
        return MyVec3f(y * v.z - z * v.y,
                       z * v.x - x * v.z,
                       x * v.y - y * v.x);
    }

    void normalize() {
        float len = sqrt(x * x + y * y + z * z);
        if (len > 1e-6) {
            x /= len; y /= len; z /= len;
        }
    }
};

vector<MyVec3f> vertices;
vector<MyVec3f> normals;
vector<MyVec3f> colors;
vector<unsigned int> indices;
Mat heightMap;

void buildMesh(float heightScale = 2.0f) {
    int rows = heightMap.rows;
    int cols = heightMap.cols;

    vertices.clear();
    indices.clear();
    normals.clear();
    colors.clear();

    float minZ = FLT_MAX, maxZ = -FLT_MAX;

    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            float z = heightMap.at<uchar>(y, x) * 0.2f;
            vertices.emplace_back(x, z, y);
            minZ = std::min(minZ, z);
            maxZ = std::max(maxZ, z);
        }
    }

    for (const auto& v : vertices) {
        float t = (v.y - minZ) / (maxZ - minZ);

      float r, g, b;
if (t < 0.25f) {
    // Lowest: Blue
    r = 0.0f;
    g = 0.0f;
    b = 1.0f;
} else if (t < 0.5f) {
    // Blue to Green
    r = 0.0f;
    g = (t - 0.25f) * 4.0f;
    b = 1.0f - (t - 0.25f) * 4.0f;
} else if (t < 0.75f) {
    // Green to Yellow
    r = (t - 0.5f) * 4.0f;
    g = 1.0f;
    b = 0.0f;
} else {
    // Yellow to Red
    r = 1.0f;
    g = 1.0f - (t - 0.75f) * 4.0f;
    b = 0.0f;
}

        colors.emplace_back(r, g, b);
    }

    for (int y = 0; y < rows - 1; ++y) {
        for (int x = 0; x < cols - 1; ++x) {
            int i = y * cols + x;
            int a = i;
            int b = i + 1;
            int c = i + cols;
            int d = i + cols + 1;
            indices.push_back(a);
            indices.push_back(c);
            indices.push_back(d);
            indices.push_back(a);
            indices.push_back(d);
            indices.push_back(b);
        }
    }

    normals.resize(vertices.size(), MyVec3f(0, 0, 0));
    for (size_t i = 0; i < indices.size(); i += 3) {
        MyVec3f &v0 = vertices[indices[i]];
        MyVec3f &v1 = vertices[indices[i + 1]];
        MyVec3f &v2 = vertices[indices[i + 2]];
        MyVec3f normal = (v1 - v0).cross(v2 - v0);
        normal.normalize();
        normals[indices[i]] = normals[indices[i]] + normal;
        normals[indices[i + 1]] = normals[indices[i + 1]] + normal;
        normals[indices[i + 2]] = normals[indices[i + 2]] + normal;
    }

    for (auto &n : normals) {
        n.normalize();
    }
}

void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    float radYaw = yaw * M_PI / 180.0f;
    float radPitch = pitch * M_PI / 180.0f;
    float dirX = cos(radPitch) * cos(radYaw);
    float dirY = sin(radPitch);
    float dirZ = cos(radPitch) * sin(radYaw);

    float targetX = camX + dirX;
    float targetY = camY + dirY;
    float targetZ = camZ + dirZ;

    gluLookAt(camX, camY, camZ, targetX, targetY, targetZ, 0, 1, 0);

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    GLfloat lightPos[] = { 400, 400, 800, 1 };
    glLightfv(GL_LIGHT0, GL_POSITION, lightPos);

    glBegin(GL_TRIANGLES);
    for (size_t i = 0; i < indices.size(); i += 3) {
        for (int j = 0; j < 3; ++j) {
            const MyVec3f &n = normals[indices[i + j]];
            const MyVec3f &v = vertices[indices[i + j]];
            const MyVec3f &c = colors[indices[i + j]];
            glColor3f(c.x, c.y, c.z);
            glNormal3f(n.x, n.y, n.z);
            glVertex3f(v.x, v.y, v.z);
        }
    }
    glEnd();

    glutSwapBuffers();
}

void reshape(int w, int h) {
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45, (float)w / h, 1, 1000);
    glMatrixMode(GL_MODELVIEW);
}

void keyboard(unsigned char key, int x, int y) {
    float radYaw = yaw * M_PI / 180.0f;
    float dirX = cos(radYaw);
    float dirZ = sin(radYaw);
    float rightX = -dirZ;
    float rightZ = dirX;

    switch (key) {
        case 27: exit(0);
        case 'w': camX += dirX * moveSpeed; camZ += dirZ * moveSpeed; break;
        case 's': camX -= dirX * moveSpeed; camZ -= dirZ * moveSpeed; break;
        case 'a': camX += rightX * moveSpeed; camZ += rightZ * moveSpeed; break;
        case 'd': camX -= rightX * moveSpeed; camZ -= rightZ * moveSpeed; break;
        case 'q': camY += moveSpeed; break;
        case 'e': camY -= moveSpeed; break;
    }
    glutPostRedisplay();
}

void specialKeys(int key, int x, int y) {
    switch (key) {
        case GLUT_KEY_LEFT: yaw -= turnSpeed; break;
        case GLUT_KEY_RIGHT: yaw += turnSpeed; break;
        case GLUT_KEY_UP: pitch = std::min(pitch + turnSpeed, 89.0f); break;
        case GLUT_KEY_DOWN: pitch = std::max(pitch - turnSpeed, -89.0f); break;
    }
    glutPostRedisplay();
}

int main(int argc, char** argv) {
    heightMap = imread(argv[1], IMREAD_GRAYSCALE);
    if (heightMap.empty()) {
        cerr << "Error loading heightmap\n";
        return -1;
    }

    cv::flip(heightMap, heightMap, 0);
    cv::equalizeHist(heightMap, heightMap);
    cv::GaussianBlur(heightMap, heightMap, cv::Size(3, 3), 0);

    imshow("Heightmap", heightMap);
    waitKey(1000);

    buildMesh();

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(800, 600);
    glutCreateWindow("3D Terrain Mesh");

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_NORMALIZE);
    glEnable(GL_COLOR_MATERIAL);

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutSpecialFunc(specialKeys);
    glutMainLoop();

    return 0;
}