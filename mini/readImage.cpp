#include <opencv2/opencv.hpp>
#include <GL/glut.h>
#include <vector>
#include <iostream>
#include <cmath>

using namespace std;
using namespace cv;

struct MyVec3f {
        float x, y, z;
        MyVec3f() : x(0), y(0), z(0) {}
        MyVec3f(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
        
    MyVec3f operator-(const MyVec3f &v) const {
        return MyVec3f(x - v.x, y - v.y, z - v.z);
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
vector<unsigned int> indices;
Mat heightMap;

// Create the mesh
void buildMesh(float heightScale = 2.0f) {
    int rows = heightMap.rows;
    int cols = heightMap.cols;

    vertices.clear();
    indices.clear();
    normals.clear();

    // Create vertices
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            float z = heightMap.at<uchar>(y, x) * heightScale;
            vertices.emplace_back(x, y, z);
        }
    }

    // Create triangles
    for (int y = 0; y < rows - 1; ++y) {
        for (int x = 0; x < cols - 1; ++x) {
            int i = y * cols + x;
            int a = i;
            int b = i + 1;
            int c = i + cols;
            int d = i + cols + 1;

            // Triangle 1: a, c, d
            indices.push_back(a);
            indices.push_back(c);
            indices.push_back(d);

            // Triangle 2: a, d, b
            indices.push_back(a);
            indices.push_back(d);
            indices.push_back(b);

        }
        std::cout << "Building triangles from "<<y << " heightmap\n";

    }

    // Compute normals
    normals.resize(vertices.size(), MyVec3f(0, 0, 0));
    for (size_t i = 0; i < indices.size(); i ++) {
        MyVec3f &v0 = vertices[indices[i]];
        MyVec3f &v1 = vertices[indices[i + 1]];
        MyVec3f &v2 = vertices[indices[i + 2]];
        MyVec3f normal = (v1 - v0).cross(v2 - v0);
        normal.normalize();

        normals[indices[i]] = normal;
        normals[indices[i + 1]] = normal;
        normals[indices[i + 2]] = normal;
    }
}

// OpenGL rendering
void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    gluLookAt(400, 300, 800,   // Eye position: directly above center
        400, 300, 0,     // Look-at position: center of the terrain
        0, 1, 0);        // Up vector: +Y is up on screen

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    GLfloat lightPos[] = { 0, 0, 300, 1 };
    glLightfv(GL_LIGHT0, GL_POSITION, lightPos);

    glBegin(GL_TRIANGLES);
    for (size_t i = 0; i < indices.size(); ++i) {
        const MyVec3f &n = normals[indices[i]];
        const MyVec3f &v = vertices[indices[i]];
        glNormal3f(n.x, n.y, n.z);
        glVertex3f(v.x, v.y, v.z);
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

// Entry point
int main(int argc, char** argv) {
    heightMap = imread(argv[1], IMREAD_GRAYSCALE);
    if (heightMap.empty()) {
        cerr << "Error loading heightmap\n";
        return -1;
    }
    cv::equalizeHist(heightMap, heightMap);

       cv::GaussianBlur(heightMap, heightMap, cv::Size(3, 3), 0);
   cv::GaussianBlur(heightMap, heightMap, cv::Size(3, 3), 0);

    imshow("Processed Heightmap", heightMap);
waitKey(1000);

    std::cout << "Image size: " << heightMap.cols << " x " << heightMap.rows << std::endl;
    std::cout << "Sample height values:\n";
    for (int y = 0; y < std::min(5, heightMap.rows); ++y) {
        for (int x = 0; x < std::min(5, heightMap.cols); ++x) {
            std::cout << (int)heightMap.at<uchar>(y, x) << " ";
        }
        std::cout << "\n";
    }


    namedWindow("Heightmap", WINDOW_NORMAL);
    imshow("Heightmap", heightMap);
    waitKey(1000); // show image for 1 second

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
    glutMainLoop();

    return 0;
}
