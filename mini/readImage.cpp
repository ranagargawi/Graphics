#include <opencv2/opencv.hpp>
#include <GL/glut.h>
#include <vector>
#include <iostream>
#include <cmath>
#include <vector>
#include <fstream>
#include <glm/glm.hpp>    
#include <filesystem>   // C++17

using namespace std;
using namespace cv;
using namespace glm;
namespace fs = std::filesystem;
std::string outputDir = "picked_points"; // Directory to save picked points

// new variables for the option to change the camera position
float camX = 400, camY = 300, camZ = 800;   // Eye position
float yaw = 0.0f;   // Horizontal angle (in degrees)
float pitch = -30.0f; // Vertical angle
float moveSpeed = 10.0f;
float turnSpeed = 5.0f;
int mainWindow, globalWindow, epnpWindow;
int activeWindow = 0;
int currScreenshot = 0;
bool showingScreenshots = false;
bool preStage = true; //starting in the pre stage, moving to the run stage after taking the first 10 pictures

vector<unsigned int> indices;
Mat heightMap;
//std::vector<int> pickedIDs;; //NEW- picking list
std::vector<std::vector<int>> pickedGlobalIDs; 
std::vector<cv::Point2f> pickedPoints2D; // store 2D pixel coordinates
std::vector<std::vector<cv::Point3f>> pickedPoints3D;  // Stores 3D coordinates of picked points
std::vector<glm::vec3> cameraTrack; //NEW - camera position track
std::vector<cv::Mat> screenshots; //NEW - saved screenshots
std::vector<std::tuple<float, float, float, float>> screenshotPositions; //NEW - screenshots positions
// the location of the screenshot that was computed by ePnP
std::vector<std::tuple<float, float, float, float>> ePnPPositions;
cv::Mat camPos;
glm::vec3 forwardVec;
glm::vec3 upVec;     
std::vector<vector<cv::KeyPoint>> imgsFeatures; 


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
    glViewport(0, 0, 1000, 800);
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
    
    
    if(preStage){
        // if we're on the pre stage, draw dots in the features location, one at a time - for the picking option
        if(imgsFeatures.size() != 0){
            GLint viewport[4];
            glGetIntegerv(GL_VIEWPORT, viewport);
            int winW = viewport[2];
            int winH = viewport[3];

            glPushAttrib(GL_ENABLE_BIT | GL_POINT_BIT | GL_COLOR_BUFFER_BIT);
            glDisable(GL_LIGHTING);
            glDisable(GL_DEPTH_TEST);
            glDisable(GL_TEXTURE_2D);
            glPointSize(10.0f); 

            glMatrixMode(GL_PROJECTION);
            glPushMatrix();
            glLoadIdentity();
            
            gluOrtho2D(0, winW, 0, winH);
            glMatrixMode(GL_MODELVIEW);
            glPushMatrix();
            glLoadIdentity();
            glBegin(GL_POINTS);
            int globalPickedNum = 0;
            if(pickedGlobalIDs.size() > imgsFeatures.size()){
                globalPickedNum = -1;
            }
            if(pickedGlobalIDs.size() > 0 && (pickedGlobalIDs.back()).size() > 0){
                globalPickedNum = (int) (pickedGlobalIDs.back()).size();
            }
            for (int idx = 0; idx < globalPickedNum + 1 && idx < 10; idx++) {
                KeyPoint keypoint = (imgsFeatures.back()).at(idx);
                // Pick color based on index
                if (idx == 0) glColor3f(0.5f, 0.0f, 0.5f); //purple
                if (idx == 1) glColor3f(0.0f, 0.5f, 0.5f); //teal
                if (idx == 2) glColor3f(0.5f, 0.0f, 0.0f); //maroon
                if (idx == 3) glColor3f(1.0f, 0.75f, 0.8f); //pink
                if (idx == 4) glColor3f(1.0f, 0.75f, 0.2f); //orange
                if(idx == 5) glColor3f(1.0f, 1.0f, 0.0f); // yellow
                if(idx == 6) glColor3f(0.0f, 1.0f, 1.0f); //light blue
                if(idx == 7) glColor3f(1.0f, 1.0f, 1.0f); //white
                if(idx == 8) glColor3f(1.0f, 0.0f, 1.0f); //Magenta
                if(idx == 9) glColor3f(0.0f, 0.5f, 0.5f); //olive green
                glVertex2f(keypoint.pt.x, keypoint.pt.y);

                
            }
        glEnd();
        glPopMatrix();
        glMatrixMode(GL_PROJECTION);
        glPopMatrix();
        glMatrixMode(GL_MODELVIEW);
        glPopAttrib();
        }

        glm::vec3 cameraPosition = glm::vec3(camX, camY, camZ);
        if (cameraTrack.empty() || glm::distance(cameraPosition, cameraTrack.back()) > 0.1f) {
            cameraTrack.push_back(cameraPosition);
            std::cout << camX << "," << camY << "," << camZ << "\n";
        
        }
        
        
    }
    
    glutSwapBuffers();
}



bool ensureOutputDir(const std::string& dir) {
    std::error_code ec;
    if (dir.empty()) return false;
    if (fs::exists(dir, ec)) return fs::is_directory(dir, ec);
    return fs::create_directories(dir, ec);
}

// Saves one file per image with the 3D coords (and matching 2D pixel if available)
void savePickedDataForAllImages() {
    if (!ensureOutputDir(outputDir)) {
        std::cerr << "Failed to create/access output directory: " << outputDir << "\n";
        return;
    }

    //size_t numImages = std::min((size_t)10, pickedPoints3D.size());

    for (size_t imgIdx = 0; imgIdx < 10; ++imgIdx) {
        // File name: img_01_points3d.csv
        char fname[256];
        std::snprintf(fname, sizeof(fname), "img_%02zu_points3d.csv", imgIdx + 1);
        fs::path outPath = fs::path(outputDir) / fname;
        std::ofstream out(outPath);
        if (!out.is_open()) {
            std::cerr << "Could not open " << outPath.string() << " for writing\n";
            continue;
        }else{
            printf("opened outputdir");
        }

        // // Optional: header
         out << "index,X,Y,Z,px,py\n";

        // 3D points we collected by global picking:
        const auto& pts3 = pickedPoints3D[imgIdx];

        // If you also want to store the pixel points used for this image, you need a per-image 2D store.
        // Right now, pickedPoints2D is a single flat vector that you clear later.
        // If you’ve mirrored 2D points per image into a vector<vector<Point2f>> (e.g., pickedPoints2DByImage),
        // you can write them side-by-side here. Otherwise we’ll write -1,-1 placeholders.

        for (size_t k = 0; k < pts3.size(); ++k) {
            const cv::Point3f& P = pts3[k];
            // If you have per-image 2D points, replace px/py with the actual values:
            float px = -1.0f, py = -1.0f;
            out << k << "," << P.x << "," << P.y << "," << P.z << "," << px << "," << py << "\n";
        }
        out.close();

        // Also save camera pose (where the screenshot was taken) if available:
        if (imgIdx < screenshotPositions.size()) {
            char poseName[256];
            std::snprintf(poseName, sizeof(poseName), "img_%02zu_pose.txt", imgIdx + 1);
            fs::path posePath = fs::path(outputDir) / poseName;
            std::ofstream pout(posePath);
            if (pout.is_open()) {
                auto [x, y, z, yawDeg] = screenshotPositions[imgIdx];
                pout << "camX " << x << "\ncamY " << y << "\ncamZ " << z << "\nyaw_deg " << yawDeg << "\n";
                pout.close();
            }
        }
    }

    std::cout << "Saved picked 3D points to folder: " << outputDir << "\n";
}


void displayGlobalView(){
    glutSetWindow(globalWindow);
    glViewport(0, 0, 1000, 800);
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
    for (size_t i = 0; i < indices.size(); i+=3) {  
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
    
    //drawing the picking global dots 
    if(preStage && !pickedGlobalIDs.empty()){
        glDisable(GL_LIGHTING);
        glPointSize(10.0f); // Bigger dots
        glBegin(GL_POINTS);
        for (size_t idx = 0; idx < pickedGlobalIDs.back().size(); idx++) {
            int id = pickedGlobalIDs.back().at(idx);
            int i = id * 3;

            // Pick color based on index
            if (idx == 0) glColor3f(0.5f, 0.0f, 0.5f); //purple
            if (idx == 1) glColor3f(0.0f, 0.5f, 0.5f); //teal
            if (idx == 2) glColor3f(0.5f, 0.0f, 0.0f); //maroon
            if (idx == 3) glColor3f(1.0f, 0.75f, 0.8f); //pink
            if (idx == 4) glColor3f(1.0f, 0.75f, 0.2f); //orange
            if(idx == 5) glColor3f(1.0f, 1.0f, 0.0f); // yellow
            if(idx == 6) glColor3f(0.0f, 1.0f, 1.0f); //light blue
            if(idx == 7) glColor3f(1.0f, 1.0f, 1.0f); //white
            if(idx == 8) glColor3f(1.0f, 0.0f, 1.0f); //Magenta
            if(idx == 9) glColor3f(0.0f, 0.5f, 0.5f); //olive green
            MyVec3f v0 = vertices[indices[i]];
            MyVec3f v1 = vertices[indices[i + 1]];
            MyVec3f v2 = vertices[indices[i + 2]];

            MyVec3f center(
                (v0.x + v1.x + v2.x) / 3.0f,
                (v0.y + v1.y + v2.y) / 3.0f,
                (v0.z + v1.z + v2.z) / 3.0f
            );
            glVertex3f(center.x, center.y, center.z);
            //check if we're on the last dot but not in the last picture
            if(idx == 9 && pickedGlobalIDs.size() < 10){
                //create new empty vector for a new picture in pickedGlobalIDs
                std::vector<int> newVec = std::vector<int>(0);
                pickedGlobalIDs.push_back(newVec);
                //create new empty vector for a new picture in pickedPoints3D
                std::vector<cv::Point3f> newPointVec = std::vector<cv::Point3f>(0);
                pickedPoints3D.push_back(newPointVec);

                glutPostWindowRedisplay(globalWindow);
                glutPostWindowRedisplay(mainWindow);
            }
            //check if we're on the last dot in the last pictures
            else if(idx == 9 && pickedGlobalIDs.size() == 10){
                printf("vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv");
                // create new file and write for each picture all it's features and their 3d location (saved on pickedPoints3D)
                    savePickedDataForAllImages();
                    printf("doneodneodoewjdjwed");

                    return;
       
            }
        }
        glEnd();
    }
    

    // in the run stage- draw the actual track line and screen shots vs the computed track line and screen shots locations
    if(!preStage){
        //drawing track line in white
        glDisable(GL_LIGHTING);
        glDisable(GL_DEPTH_TEST);
        glColor3f(1.0f, 1.0f, 1.0f); 
        glLineWidth(5.0f);
        glBegin(GL_LINE_STRIP);
        for (const auto& pos : cameraTrack) {
            glVertex3f(pos.x, pos.y, pos.z);
        }
        glEnable(GL_DEPTH_TEST);
        glEnd();
        glEnable(GL_LIGHTING);

        //drawing the screenshots position triangle in white
        glDisable(GL_LIGHTING);
        glDisable(GL_DEPTH_TEST);
        glColor3f(1.0f, 1.0f, 1.0f);
        float size = 20.0f;             // Triangle size
        for (const auto& pos : screenshotPositions) {
            float x = std::get<0>(pos);
            float y = std::get<1>(pos);
            float z = std::get<2>(pos);
            float cameraYaw = std::get<3>(pos);
            
            glPushMatrix();
            glTranslatef(x, y + 0.1f, z);  // slightly above ground

            // Rotate so triangle faces camera
            glRotatef(-cameraYaw , 0.0f, 1.0f, 0.0f);

            glBegin(GL_TRIANGLES);
            // Triangle pointing "forward" (positive Z after rotation)
            glVertex3f(0.0f, 0.0f, size);
            glVertex3f(-size, 0.0f, -size);
            glVertex3f(size, 0.0f, -size);
            glEnd();

            glPopMatrix();
        }
        glEnable(GL_LIGHTING);
        glEnable(GL_DEPTH_TEST);

        //drawing the ePnP positions in red
        glDisable(GL_LIGHTING);
        glDisable(GL_DEPTH_TEST);
        glColor3f(1.0f, 0.0f, 0.0f);
        size = 20.0f;             // Triangle size
        for (const auto& pos : ePnPPositions) {
            float x = std::get<0>(pos) ;
            float y = std::get<1>(pos) ;
            float z = std::get<2>(pos);
            float cameraYaw = std::get<3>(pos);

            glPushMatrix();
            glTranslatef(x, y + 0.1f, z);  // slightly above ground

            // Convert yaw from radians to degrees for OpenGL
            float cameraYawDeg = cameraYaw * 180.0f / M_PI;
            // Rotate so triangle faces camera
            glRotatef(-cameraYawDeg , 0.0f, 1.0f, 0.0f);

            glBegin(GL_TRIANGLES);
            // Triangle pointing "forward" (positive Z after rotation)
            glVertex3f(0.0f, 0.0f, size);
            glVertex3f(-size, 0.0f, -size);
            glVertex3f(size, 0.0f, -size);
            glEnd();

            glPopMatrix();
        }
        glEnable(GL_LIGHTING);
        glEnable(GL_DEPTH_TEST); 

        //TODO: add a line between the computed epnp positions
    }
    
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

void renderForPickingGlobal() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, 800.0 / 600.0, 1.0, 2000.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(850, 850, 2000, 600, 200, 1200, 0, 1, 0);

    glDisable(GL_LIGHTING);

    glBegin(GL_TRIANGLES);
    for (size_t i = 0; i < indices.size(); i += 3) {
        int triID = i / 3;
        GLubyte r = triID & 0xFF;
        GLubyte g = (triID >> 8) & 0xFF;
        GLubyte b = (triID >> 16) & 0xFF;
        glColor3ub(r, g, b);

        for (int j = 0; j < 3; ++j) {
            const MyVec3f &v = vertices[indices[i + j]];
            glVertex3f(v.x, v.y, v.z);
        }
    }
    glEnd();
}


// mouse click option for picking
void mouseClick(int button, int state, int x, int y) {

    //picking in the global window
    if (glutGetWindow() == globalWindow && button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
        GLint viewport[4];
        glGetIntegerv(GL_VIEWPORT, viewport);

        glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);

        renderForPickingGlobal();  // Draw the scene using ID colors

        // Read the pixel under the mouse
        GLubyte pixel[4];
        glReadPixels(x, viewport[3] - y - 1, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, pixel);

        int id = pixel[0] + pixel[1] * 256 + pixel[2] * 256 * 256;

        //if we're trying to pick the first dot in a picture but didn't take the picture yet- don't do anything 
        if(pickedGlobalIDs.size() > imgsFeatures.size()){ 
    
        }
        //check if we're not in the first picture, the picked id is valid and the number of picked global ids is not larger then the number of picked ids (the regular case)
        else if (id >= 0  && !pickedGlobalIDs.empty() && pickedGlobalIDs.back().size() < (imgsFeatures.back()).size()) {
            (pickedGlobalIDs.back()).push_back(id);

            //save the 3d location of the picking for ePnP
            int i = id * 3;
            MyVec3f v0 = vertices[indices[i]];
            MyVec3f v1 = vertices[indices[i + 1]];
            MyVec3f v2 = vertices[indices[i + 2]];

            glm::vec3 center3D(
                (v0.x + v1.x + v2.x) / 3.0f,
                (v0.y + v1.y + v2.y) / 3.0f,
                (v0.z + v1.z + v2.z) / 3.0f
            );
            pickedPoints3D.back().push_back(cv::Point3f(center3D.x, center3D.y, center3D.z)); 
        
        }
        //if we're picking the first dot in the first picture- create new vectors and save the new picking dot
        else if(id >= 0 && pickedGlobalIDs.size() == 0){
            //create new vector for the first picture in pickedGlobalIDs
            std::vector<int> newVec = std::vector<int>(1, id);
            pickedGlobalIDs.push_back(newVec);
            
            //save the 3d location of the picking for ePnP
            int i = id * 3;
            MyVec3f v0 = vertices[indices[i]];
            MyVec3f v1 = vertices[indices[i + 1]];
            MyVec3f v2 = vertices[indices[i + 2]];

            glm::vec3 center3D(
                (v0.x + v1.x + v2.x) / 3.0f,
                (v0.y + v1.y + v2.y) / 3.0f,
                (v0.z + v1.z + v2.z) / 3.0f
            );
            //create new vector for the first picture in pickedPoints3D
            std::vector<cv::Point3f> newPointVec = std::vector<cv::Point3f>(0);
            pickedPoints3D.push_back(newPointVec);
            pickedPoints3D.back().push_back(cv::Point3f(center3D.x, center3D.y, center3D.z)); 
        }
        // the case where we're picking the last dot in a picture will be handled in the display global function
        glutPostWindowRedisplay(globalWindow);
        glutPostWindowRedisplay(mainWindow);
        glutPostRedisplay(); // Ask OpenGL to redraw the scene
    }
}

//keyboard function to move around - copyed from chatgpt
void keyboard(unsigned char key, int x, int y) {
    float radYaw = yaw * M_PI / 180.0f;
    float dirX = cos(radYaw);
    float dirZ = sin(radYaw);
    float rightX = -dirZ;
    float rightZ = dirX;
    if(glutGetWindow() != mainWindow)
        glutSetWindow(mainWindow);

    if(!showingScreenshots && key == 27){
        exit(0);
    }
    switch (key) {
    case 'w':
        camX += dirX * moveSpeed;
        camZ += dirZ * moveSpeed;
        break;
    case 's':
        camX -= dirX * moveSpeed;
        camZ -= dirZ * moveSpeed;
        break;
    case 'd':
        camX += rightX * moveSpeed;
        camZ += rightZ * moveSpeed;
        break;
    case 'a':
        camX -= rightX * moveSpeed;
        camZ -= rightZ * moveSpeed;
        break;
    case 'q':
        camY += moveSpeed;
        break;
    case 'e':
        camY -= moveSpeed;
        break;
    // taking screenshots of the current position in the main window and showing the location of the screenshot taken in the global window
    case 'B':
    case 'b': {
        GLint viewport[4];
        glGetIntegerv(GL_VIEWPORT, viewport);
        int width = viewport[2], height = viewport[3];

        std::vector<GLubyte> pixels(width * height * 3);
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());

        cv::Mat img(height, width, CV_8UC3, pixels.data());
        cv::Mat flipped;
        cv::flip(img, flipped, 0); 

        cv::Mat bgr;
        cv::cvtColor(flipped, bgr, cv::COLOR_RGB2BGR);
        screenshots.push_back(bgr.clone());  
        screenshotPositions.emplace_back(camX, camY, camZ, yaw);
        std::cout << "Screenshot taken at:" << camX << "," << camY << "," << camZ << " Total saved: " << screenshots.size() << "\n";
        
        //TODO: in the pre stage - for each screenshot, compute features and save them in a global list.
        cv::Ptr<cv::ORB> orb = cv::ORB::create();
        std::vector<cv::KeyPoint> features;
        cv::Mat descriptors;
        orb->detectAndCompute(img, cv::noArray(), features, descriptors);

        std::sort(features.begin(), features.end(),
          [](const cv::KeyPoint &a, const cv::KeyPoint &b) {
              return a.response > b.response; 
          });
        std::vector<cv::KeyPoint> selected;
        float minDist = 30.0; 
        for (auto &kp : features) {
            bool good = true;
            for (auto &sel : selected) {
                if (cv::norm(kp.pt - sel.pt) < minDist) {
                    good = false;
                    break;
                }
            }
            if (good) selected.push_back(kp);
            if (selected.size() == 10) break;
        }
    
        cv::Mat outImg;
        cv::drawKeypoints(img, selected, outImg, cv::Scalar(0,255,0));
        cv::Mat flipped2;
        cv::flip(outImg, flipped2, 0); 
        cv::Mat fixedImg;
        cv::cvtColor(flipped2, fixedImg, cv::COLOR_RGB2BGR);
        cv::imshow("Features", fixedImg);
        imgsFeatures.push_back(selected);
        //after 10 screenshots, move to the run stage
        if(imgsFeatures.size() == 10){
            preStage = false;
        }
        int key = cv::waitKey(0);
        showingScreenshots = false;
        cv::destroyWindow("Features");
        if (key ==27){ // ESC
                showingScreenshots = false;
                cv::destroyWindow("Features");
                break;
        }
        
        
        break;
    }
    // showing the screenshots - TODO: check if need to be removed
    case 'R': {
        if (!screenshots.empty()) {
            showingScreenshots = true;
            currScreenshot = 0;
            while (showingScreenshots) {
                cv::imshow("Screenshot", screenshots[currScreenshot]);
                int key = cv::waitKey(0);
                switch (key) {
                    case 27: // ESC
                        showingScreenshots = false;
                        cv::destroyWindow("Screenshot");
                        break;
                    case 81: // Left
                        if (currScreenshot > 0) currScreenshot--;
                        break;
                    case 83: // Right
                        if (currScreenshot < (int)screenshots.size() - 1) currScreenshot++;
                        break;
                }
            }
        }
        break;
    }
    // computing ePnP
    case 'C':
    case 'c': {
        // only on the run stage
        if(!preStage){
            if(pickedPoints2D.size() < 4 || pickedPoints3D.size() < 4){
                cout << "not enough points picked" << "\n";
                break;
            }

            //TODO: check which picture we have the most matching features with, compute epnp with the features.     
            cv::Mat rvec, tvec;
            int width = 1000, height = 800;
            double fovy = 45.0 * CV_PI / 180.0; // radians
            double fy = height / (2.0 * tan(fovy / 2.0));
            double fx = fy; // square pixels
            double cx = width / 2.0;
            double cy = height / 2.0;

            cv::Mat cameraMatrix = (cv::Mat_<double>(3,3) << fx, 0, cx,
                                                            0, fy, cy,
                                                            0, 0, 1);

            cv::solvePnP(pickedPoints3D, pickedPoints2D, cameraMatrix, cv::Mat::zeros(4, 1, CV_64F), rvec, tvec, false, cv::SOLVEPNP_EPNP);

            cv::Mat R;
            cv::Rodrigues(rvec, R);  // convert rvec to 3x3 rotation matrix
            cv::Mat Rt = R.t();      // transpose
            camPos = -Rt * tvec; // camera position in world coords
            glm::vec3 forwardVec(R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2));
            glm::vec3 upVec(R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2));

            double yaw = atan2(R.at<double>(1,0), R.at<double>(0,0));
            ePnPPositions.emplace_back(camPos.at<double>(0), camPos.at<double>(1), camPos.at<double>(2), yaw);
            std::cout << "ePnP camPos: " 
            << camPos.at<double>(0) << ", "
            << camPos.at<double>(1) << ", "
            << camPos.at<double>(2) << "\n";

            //delete the picked points after the computation - TODO: update!
            pickedPoints2D.clear();
            pickedPoints3D.clear();
            //pickedIDs.clear();
            pickedGlobalIDs.clear();
            }
        //check if we need to display a new window of the world from the computed ePnP angles 
    }
    }

    glutPostRedisplay();  // Moved here
}
//end of Chatgpt

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

    std::cout << "Image size: " << heightMap.cols << " x " << heightMap.rows << std::endl;
    // std::cout << "Sample height values:\n";
    // for (int y = 0; y < std::min(5, heightMap.rows); ++y) {
    //     for (int x = 0; x < std::min(5, heightMap.cols); ++x) {
    //         std::cout << (int)heightMap.at<uchar>(y, x) << " ";
    //     }
    //     std::cout << "\n";
    // }


    namedWindow("Heightmap", WINDOW_NORMAL);
    // imshow("Heightmap", heightMap);
    // waitKey(1000); // show image for 1 second

    buildMesh();

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(1000, 800);
    glutInitWindowPosition(1000, 50); // Adjust as needed
    mainWindow = glutCreateWindow("3D Terrain Mesh");

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_NORMALIZE);
    glEnable(GL_COLOR_MATERIAL);

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutMouseFunc(mouseClick);
    glutKeyboardFunc(keyboard);
    glutSpecialFunc(specialKeys);

    glutInitWindowPosition(1100, 50); // Shifted right to be side by side (800 + margin)

    globalWindow = glutCreateWindow("global View");
    glutDisplayFunc(displayGlobalView);
    glutMouseFunc(mouseClick); 
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