#include <opencv2/opencv.hpp>
#include <GL/glut.h>
#include <vector>
#include <iostream>
#include <cmath>
#include <vector>
#include <fstream>
#include <glm/glm.hpp>    

using namespace std;
using namespace cv;
using namespace glm;

// new variables for the option to change the camera position
float camX = 400, camY = 300, camZ = 800;   // Eye position
float yaw = 0.0f;   // Horizontal angle (in degrees)
float pitch = -30.0f; // Vertical angle
float moveSpeed = 10.0f;
float turnSpeed = 5.0f;
int mainWindow, globalWindow;
int activeWindow = 0;


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
std::vector<glm::vec3> cameraTrack; //NEW - camera position track
//glm::vec3 cameraPosition;

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
        //std::cout << "Building triangles from "<<y << " heightmap\n";

    }

    // Compute normals
     normals.resize(vertices.size(), MyVec3f(0, 0, 0));
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
    glutSetWindow(mainWindow);
    glViewport(0, 0, 640, 480);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    float radYaw = yaw * M_PI / 180.0f;
    float radPitch = pitch * M_PI / 180.0f;
    float dirX = cos(radPitch) * cos(radYaw);
    float dirY = sin(radPitch);
    float dirZ = cos(radPitch) * sin(radYaw);

    // Target the camera is looking at
    float targetX = camX + dirX;
    float targetY = camY + dirY;
    float targetZ = camZ + dirZ;

    gluLookAt(camX, camY, camZ, targetX, targetY, targetZ, 0, 1, 0);
    
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    GLfloat lightPos[] = { 400, 400, 800, 1 }; //CHANGED

    glLightfv(GL_LIGHT0, GL_POSITION, lightPos);

    

    glBegin(GL_TRIANGLES);
    for (size_t i = 0; i < indices.size(); i+=3) {// chat gpt changed it to be i+=3
            for (int j = 0; j < 3; ++j) {
                const MyVec3f &v = vertices[indices[i + j]];
                const MyVec3f &n = normals[indices[i + j]];

                // Normalize height to [0, 1] for color mapping
                float t = v.y / 51.0f; // or adjust if scaled differently

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

                glColor3f(r, g, b);

                glNormal3f(n.x, n.y, n.z);
                glVertex3f(v.x, v.y, v.z);
            
        }
        
    }


    
    glEnd();

    glDisable(GL_LIGHTING);
    glColor3f(1.0f, 1.0f, 1.0f); 
    glPointSize(10.0f); // Bigger dots
    glBegin(GL_POINTS);
    for (int id : pickedIDs) {
        int i = id * 3;
        MyVec3f v0 = vertices[indices[i]];
        MyVec3f v1 = vertices[indices[i + 1]];
        MyVec3f v2 = vertices[indices[i + 2]];
        MyVec3f center = MyVec3f(
            (v0.x + v1.x + v2.x) / 3.0f,
            (v0.y + v1.y + v2.y) / 3.0f,
            (v0.z + v1.z + v2.z) / 3.0f
        );
        glVertex3f(center.x, center.y, center.z);
    }
    glEnd();
    glEnable(GL_LIGHTING);

    glm::vec3 cameraPosition = glm::vec3(camX, camY, camZ);
    if (cameraTrack.empty() || glm::distance(cameraPosition, cameraTrack.back()) > 0.1f) {
        cameraTrack.push_back(cameraPosition);
       // std::cout << camX << "," << camY << "," << camZ << "\n";
    
    }
    
    glutSwapBuffers();
}

void displayGlobalView(){
    glutSetWindow(globalWindow);
    glViewport(0, 0, 640, 480);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, 800.0 / 600.0, 1.0, 2000.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(850, 850, 2000, 600, 200, 1200, 0, 1, 0);

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);

    GLfloat lightPos[] = { 400, 1000, 800, 1.0f };

    glLightfv(GL_LIGHT0, GL_POSITION, lightPos);
    glBegin(GL_TRIANGLES);
    for (size_t i = 0; i < indices.size(); i+=3) {// chat gpt changed it to be i+=3    
            for (int j = 0; j < 3; ++j) {
                const MyVec3f &v = vertices[indices[i + j]];
                const MyVec3f &n = normals[indices[i + j]];

                // Normalize height to [0, 1] for color mapping
                float t = v.y / 51.0f; // or adjust if scaled differently

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

                glColor3f(r, g, b);

                glNormal3f(n.x, n.y, n.z);
                glVertex3f(v.x, v.y, v.z);
            }
        
        
    }

    glEnd();
    
    glDisable(GL_LIGHTING);
    
    glColor3f(1.0f, 1.0f, 1.0f);
    glPointSize(10.0f); // Bigger dots
    glBegin(GL_POINTS);
    for (int id : pickedIDs) {
        int i = id * 3;
        MyVec3f v0 = vertices[indices[i]];
        MyVec3f v1 = vertices[indices[i + 1]];
        MyVec3f v2 = vertices[indices[i + 2]];
        MyVec3f center = MyVec3f(
            (v0.x + v1.x + v2.x) / 3.0f,
            (v0.y + v1.y + v2.y) / 3.0f,
            (v0.z + v1.z + v2.z) / 3.0f
        );
        glVertex3f(center.x, center.y, center.z);
    }
    glEnd();

   // Draw camera path
glDisable(GL_LIGHTING);   // Ensure lighting doesn't dim the line
glDisable(GL_DEPTH_TEST); // Prevent line from being occluded
glLineWidth(80.0f);         // Adjust width as desired
glColor3f(1.0f, 1.0f, 1.0f); // Bright white

glBegin(GL_LINE_STRIP);
for (const auto& pos : cameraTrack) {
    glVertex3f(pos.x, pos.y, pos.z);
}
glEnd();

glEnable(GL_DEPTH_TEST);  // Restore default state
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


// mouse click option for picking
void mouseClick(int button, int state, int x, int y) {
    if (glutGetWindow() == mainWindow && button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
        GLint viewport[4];
        glGetIntegerv(GL_VIEWPORT, viewport);

        glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);

        renderForPicking();  // Draw the scene using ID colors

        // Read the pixel under the mouse
        GLubyte pixel[4];
        glReadPixels(x, viewport[3] - y - 1, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, pixel);

        int id = pixel[0] + pixel[1] * 256 + pixel[2] * 256 * 256;

        // Add the picked id only if valid and not already picked
        if (id >= 0 && std::find(pickedIDs.begin(), pickedIDs.end(), id) == pickedIDs.end()) {
            pickedIDs.push_back(id);
            glutPostWindowRedisplay(globalWindow);
        
        }
        glutPostRedisplay(); // Ask OpenGL to redraw the scene
    }
}

//keyboard function to move around - copyed from chatgpt
void keyboard(unsigned char key, int x, int y) {
    float radYaw = yaw * M_PI / 180.0f;
    //float radPitch = pitch * M_PI / 180.0f;

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

    // imshow("Processed Heightmap", heightMap);
    // waitKey(1000);

   // std::cout << "Image size: " << heightMap.cols << " x " << heightMap.rows << std::endl;
    // std::cout << "Sample height values:\n";
    // for (int y = 0; y < std::min(5, heightMap.rows); ++y) {
    //     for (int x = 0; x < std::min(5, heightMap.cols); ++x) {
    //         std::cout << (int)heightMap.at<uchar>(y, x) << " ";
    //     }
    //     std::cout << "\n";
    // }


    // namedWindow("Heightmap", WINDOW_NORMAL);
    // imshow("Heightmap", heightMap);
    // waitKey(1000); // show image for 1 second

    buildMesh();

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(640, 480);
    glutInitWindowPosition(100, 100); // Adjust as needed
    mainWindow = glutCreateWindow("3D Terrain Mesh");

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_NORMALIZE);
    glEnable(GL_COLOR_MATERIAL);

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutMouseFunc(mouseClick);
    glutKeyboardFunc(keyboard);
    glutSpecialFunc(specialKeys);

    glutInitWindowPosition(760, 100); // Shifted right to be side by side (800 + margin)

    globalWindow = glutCreateWindow("global View");
    glutDisplayFunc(displayGlobalView);
    glutIdleFunc([]() {
        glutPostRedisplay();
        glutPostWindowRedisplay(globalWindow);
    });
    glutMainLoop();

    // //printing the track positions
    // for (const auto& pos : cameraTrack) {
    //     std::cout << pos.x << "," << pos.y << "," << pos.z << "\n";
    // }
    

    return 0;
}