#include <opencv2/opencv.hpp>
#include <GL/glut.h>
#include <vector>
#include <iostream>
#include <cmath>

using namespace std;
using namespace cv;
// new variables for the option to change the camera position
float camX = 400, camY = 300, camZ = 800;   // Eye position
float yaw = 0.0f;   // Horizontal angle (in degrees)
float pitch = -30.0f; // Vertical angle
float moveSpeed = 10.0f;
float turnSpeed = 5.0f;

struct MyVec3f {
        float x, y, z;
        MyVec3f() : x(0), y(0), z(0) {}
        MyVec3f(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
        
    MyVec3f operator-(const MyVec3f &v) const {
        return MyVec3f(x - v.x, y - v.y, z - v.z);
    }

    MyVec3f operator+(const MyVec3f &v) const { //CHANGED
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
vector<unsigned int> indices;
Mat heightMap;
std::vector<int> pickedIDs;; //NEW- picking list

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
            //float z = heightMap.at<uchar>(y, x) * heightScale;
            float z = heightMap.at<uchar>(y, x) * 0.2f; //CHANGED

            //vertices.emplace_back(x, y, z);
            vertices.emplace_back(x, z, y); // CHANGED
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
    // for (size_t i = 0; i < indices.size(); i ++) {
    //     MyVec3f &v0 = vertices[indices[i]];
    //     MyVec3f &v1 = vertices[indices[i + 1]];
    //     MyVec3f &v2 = vertices[indices[i + 2]];
    //     MyVec3f normal = (v1 - v0).cross(v2 - v0);
    //     normal.normalize();

    //     normals[indices[i]] = normal;
    //     normals[indices[i + 1]] = normal;
    //     normals[indices[i + 2]] = normal;

    // }
    //CHANGED
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

    // Then normalize all
    for (auto &n : normals) {
        n.normalize();
    }
}

// OpenGL rendering
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
            int vertexIdx = indices[i + j];
            const MyVec3f &v = vertices[vertexIdx];
            const MyVec3f &n = normals[vertexIdx];

            if (std::find(pickedIDs.begin(), pickedIDs.end(), vertexIdx) != pickedIDs.end()) {
                continue; // skip this, draw later as a square
            }

            float t = v.y / 51.0f;
            float r, g, b;
            if (t < 0.25f) {
                r = 0.0f; g = 0.0f; b = 1.0f;
            } else if (t < 0.5f) {
                r = 0.0f; g = (t - 0.25f) * 4.0f; b = 1.0f - (t - 0.25f) * 4.0f;
            } else if (t < 0.75f) {
                r = (t - 0.5f) * 4.0f; g = 1.0f; b = 0.0f;
            } else {
                r = 1.0f; g = 1.0f - (t - 0.75f) * 4.0f; b = 0.0f;
            }

            glColor3f(r, g, b);
            glNormal3f(n.x, n.y, n.z);
            glVertex3f(v.x, v.y, v.z);
        }
    }
    glEnd();

    // Draw picked vertices as white squares
    float size = 2.0f;
    glDisable(GL_LIGHTING);
    glBegin(GL_QUADS);
    glColor3f(1.0f, 1.0f, 1.0f);
    for (int idx : pickedIDs) {
        const MyVec3f &v = vertices[idx];
        glVertex3f(v.x - size, v.y, v.z - size);
        glVertex3f(v.x + size, v.y, v.z - size);
        glVertex3f(v.x + size, v.y, v.z + size);
        glVertex3f(v.x - size, v.y, v.z + size);
    }
    glEnd();
    glEnable(GL_LIGHTING);

    glutSwapBuffers();
}


void reshape(int w, int h) {
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45, (float)w / h, 1, 1000);
    glMatrixMode(GL_MODELVIEW);
}

//NEW - picking option
void renderForPicking() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    //gluLookAt(400, 300, 800, 400, 300, 0, 0, 1, 0); //constant camera view version

    float radYaw = yaw * M_PI / 180.0f;
    float radPitch = pitch * M_PI / 180.0f;
    float dirX = cos(radPitch) * cos(radYaw);
    float dirY = sin(radPitch);
    float dirZ = cos(radPitch) * sin(radYaw);

    gluLookAt(camX, camY, camZ, 
              camX + dirX, camY + dirY, camZ + dirZ, 
              0, 1, 0);

    glDisable(GL_LIGHTING);

    glBegin(GL_TRIANGLES);
    for (size_t i = 0; i < indices.size(); i += 3) {
        int id = i / 3;
        int r = (id & 0x000000FF);
        int g = (id & 0x0000FF00) >> 8;
        int b = (id & 0x00FF0000) >> 16;
        glColor3ub(r, g, b);

        for (int j = 0; j < 3; ++j) {
            const MyVec3f &v = vertices[indices[i + j]];
            glVertex3f(v.x, v.y, v.z);
        }
    }
    glEnd();
    glFlush(); // Makes sure all commands are finished
}



void mouseClick(int button, int state, int x, int y) {
    if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
        GLint viewport[4];
        glGetIntegerv(GL_VIEWPORT, viewport);

        renderForPicking();

        GLubyte pixel[4];
        glReadPixels(x, viewport[3] - y, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, pixel);

        int triID = pixel[0] + pixel[1] * 256 + pixel[2] * 256 * 256;

        if (triID >= 0 && triID * 3 + 2 < indices.size()) {
            for (int j = 0; j < 3; ++j) {
                int vIdx = indices[triID * 3 + j];
                if (std::find(pickedIDs.begin(), pickedIDs.end(), vIdx) == pickedIDs.end()) {
                    pickedIDs.push_back(vIdx);
                }
            }
        }

        glutPostRedisplay();
    }
}

//keyboard function to move around - copyed from chatgpt
void keyboard(unsigned char key, int x, int y) {
    float radYaw = yaw * M_PI / 180.0f;
    float radPitch = pitch * M_PI / 180.0f;

    // Forward direction (ignores pitch for flat movement)
    float dirX = cos(radYaw);
    float dirZ = sin(radYaw);
    float rightX = -dirZ;
    float rightZ = dirX;

    switch (key) {
    case 27: // ESC
        exit(0);
    case 'w':
        camX += dirX * moveSpeed;
        camZ += dirZ * moveSpeed;
        break;
    case 's':
        camX -= dirX * moveSpeed;
        camZ -= dirZ * moveSpeed;
        break;
    case 'a':
        camX += rightX * moveSpeed;
        camZ += rightZ * moveSpeed;
        break;
    case 'd':
        camX -= rightX * moveSpeed;
        camZ -= rightZ * moveSpeed;
        break;
    case 'q': // Up
        camY += moveSpeed;
        break;
    case 'e': // Down
        camY -= moveSpeed;
        break;
    }
    glutPostRedisplay();
}

void specialKeys(int key, int x, int y) {
    switch (key) {
    case GLUT_KEY_LEFT:
        yaw -= turnSpeed;
        break;
    case GLUT_KEY_RIGHT:
        yaw += turnSpeed;
        break;
    case GLUT_KEY_UP:
        pitch += turnSpeed;
        if (pitch > 89.0f) pitch = 89.0f;
        break;
    case GLUT_KEY_DOWN:
        pitch -= turnSpeed;
        if (pitch < -89.0f) pitch = -89.0f;
        break;
    }
    glutPostRedisplay();
}//end of Chatgpt


// Entry point
int main(int argc, char** argv) {
    heightMap = imread(argv[1], IMREAD_GRAYSCALE);
    if (heightMap.empty()) {
        cerr << "Error loading heightmap\n";
        return -1;
    }

    cv::flip(heightMap, heightMap, 0); //CHANGED - ADDED

    cv::equalizeHist(heightMap, heightMap);

       //cv::GaussianBlur(heightMap, heightMap, cv::Size(3, 3), 0);
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
    glutMouseFunc(mouseClick);
    glutKeyboardFunc(keyboard);
    glutSpecialFunc(specialKeys);
    glutMainLoop();

    return 0;
}
