// Unstructured Terrain Generation using Delaunay Triangulation with Full Renderer
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <windows.h>
#include <gl/gl.h>
#include <gl/glu.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <map>
#include <algorithm>

//this will be the mesh structure
struct Vertex { float x, y, z; };
std::vector<Vertex> terrainVertices; //holds 3d points
glm::vec3 cameraPos   = glm::vec3(0.0f, 0.0f, 3.0f);
glm::vec3 cameraFront = glm::vec3(0.0f, 0.0f, -1.0f);
glm::vec3 cameraUp    = glm::vec3(0.0f, 1.0f, 0.0f);
float yaw = -90.0f, pitch = 0.0f;
float speed = 20.0f;
float lastX = 512, lastY = 384;
bool firstMouse = true;

std::vector<unsigned int> terrainIndices; //defines how to connect tthe points via triangles
int heightmapWidth = 0, heightmapHeight = 0;
int mouseX = 0;
int mouseY = 0;
GLuint pickedID = -1;
//////////////////////////////////


//build the 3d terrain from grayscale image
void LoadUnstructuredTerrain(const std::string& path, int sampleCount = 500) {
    cv::Mat heightmap = cv::imread(path, cv::IMREAD_GRAYSCALE);
    if (heightmap.empty()) {
        std::cerr << "Failed to load heightmap: " << path << std::endl;
        return;
    }

    heightmapWidth = heightmap.cols;
    heightmapHeight = heightmap.rows;

    terrainVertices.clear();
    terrainIndices.clear();

    std::vector<cv::Point2f> samplePoints;
    cv::RNG rng;
    for (int i = 0; i < sampleCount; ++i) {
        float x = rng.uniform(0.0f, (float)heightmapWidth);
        float y = rng.uniform(0.0f, (float)heightmapHeight);
        samplePoints.emplace_back(x, y);
    }

    cv::Subdiv2D subdiv(cv::Rect(0, 0, heightmapWidth, heightmapHeight));
    for (const auto& pt : samplePoints) subdiv.insert(pt);

    std::vector<cv::Vec6f> triangleList;
    subdiv.getTriangleList(triangleList);

    std::map<std::pair<int, int>, int> vertexIndexMap;
    auto getIndex = [&](int x, int y) -> int {
        auto key = std::make_pair(x, y);
        auto it = vertexIndexMap.find(key);
        if (it != vertexIndexMap.end()) return it->second;

        uchar gray = heightmap.at<uchar>(std::clamp(y, 0, heightmap.rows - 1), std::clamp(x, 0, heightmap.cols - 1));
        float z = (gray / 255.0f) * 80.0f;
        int idx = terrainVertices.size();
        terrainVertices.push_back({ (float)x, (float)-y, z });
        vertexIndexMap[key] = idx;
        return idx;
    };

    for (const auto& t : triangleList) {
        int x1 = static_cast<int>(t[0]), y1 = static_cast<int>(t[1]);
        int x2 = static_cast<int>(t[2]), y2 = static_cast<int>(t[3]);
        int x3 = static_cast<int>(t[4]), y3 = static_cast<int>(t[5]);

        if (x1 < 0 || x1 >= heightmapWidth || y1 < 0 || y1 >= heightmapHeight) continue;
        if (x2 < 0 || x2 >= heightmapWidth || y2 < 0 || y2 >= heightmapHeight) continue;
        if (x3 < 0 || x3 >= heightmapWidth || y3 < 0 || y3 >= heightmapHeight) continue;

        int i1 = getIndex(x1, y1);
        int i2 = getIndex(x2, y2);
        int i3 = getIndex(x3, y3);

        terrainIndices.push_back(i1);
        terrainIndices.push_back(i2);
        terrainIndices.push_back(i3);
    }
}
void PerformPicking(int x, int y) {
    GLuint selectBuf[512];
    GLint hits, viewport[4];

    glSelectBuffer(512, selectBuf);
    glGetIntegerv(GL_VIEWPORT, viewport);
    glRenderMode(GL_SELECT);

    glInitNames();
    glPushName(0);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();

    gluPickMatrix((GLdouble)x, (GLdouble)(viewport[3] - y), 5.0, 5.0, viewport);
    gluPerspective(45.0f, (GLfloat)viewport[2] / viewport[3], 0.1f, 1000.0f);
    glMatrixMode(GL_MODELVIEW);

    for (size_t i = 0; i < terrainIndices.size(); i += 3) {
        glLoadName((GLuint)i);
        glBegin(GL_TRIANGLES);
        for (int j = 0; j < 3; ++j) {
            Vertex& v = terrainVertices[terrainIndices[i + j]];
            glVertex3f(v.x, v.y, v.z);
        }
        glEnd();
    }

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);

    hits = glRenderMode(GL_RENDER);
    if (hits > 0) {
        pickedID = selectBuf[3];
        std::cout << "Picked triangle starting index: " << pickedID << std::endl;
    }
}
void SetupOpenGL() {
    glShadeModel(GL_SMOOTH);
glClearColor(0.3f, 0.3f, 0.3f, 1.0f);
    glClearDepth(1.0f);
    glEnable(GL_DEPTH_TEST); //initialize openGL with Z-buffeting
    glDepthFunc(GL_LEQUAL);
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
}

void ResizeGL(int width, int height) {
    if (height == 0) height = 1;
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0f, (GLfloat)width / height, 0.1f, 1000.0f);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void RenderScene() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glMatrixMode(GL_MODELVIEW);
glLoadIdentity();
glm::mat4 view = glm::lookAt(cameraPos, cameraPos + cameraFront, cameraUp);
glLoadMatrixf(&view[0][0]);
glScalef(0.5f, 0.5f, 0.5f); // apply scaling after camera


    auto setColorFromHeight = [](float h) {
        float maxHeight = 50.0f; // must match the Z scale in getIndex
        float t = std::clamp(h / maxHeight, 0.0f, 1.0f);
        float r = t;
        float g = 1.0f - fabs(t - 0.5f) * 2.0f;
        float b = 1.0f - t;
        glColor3f(r, g, b);
    };

    //drow filled triangles
    glBegin(GL_TRIANGLES);
    for (size_t i = 0; i < terrainIndices.size(); i += 3) {
        Vertex v1 = terrainVertices[terrainIndices[i]];
        Vertex v2 = terrainVertices[terrainIndices[i + 1]];
        Vertex v3 = terrainVertices[terrainIndices[i + 2]];

        setColorFromHeight(v1.z);
        glVertex3f(v1.x, v1.y, v1.z);
        setColorFromHeight(v2.z);
        glVertex3f(v2.x, v2.y, v2.z);
        setColorFromHeight(v3.z);
        glVertex3f(v3.x, v3.y, v3.z);
    }
    glEnd();

    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glColor3f(0.0f, 0.0f, 0.0f);
    glBegin(GL_TRIANGLES);
    for (size_t i = 0; i < terrainIndices.size(); i += 3) {
        Vertex v1 = terrainVertices[terrainIndices[i]];
        Vertex v2 = terrainVertices[terrainIndices[i + 1]];
        Vertex v3 = terrainVertices[terrainIndices[i + 2]];
        glVertex3f(v1.x, v1.y, v1.z);
        glVertex3f(v2.x, v2.y, v2.z);
        glVertex3f(v3.x, v3.y, v3.z);
    }
    glEnd();
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    glFlush();
}

LRESULT CALLBACK WndProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam) {
    switch (msg) {
        case WM_SIZE:
            ResizeGL(LOWORD(lParam), HIWORD(lParam));
            return 0;
        case WM_MOUSEWHEEL: {
    short delta = GET_WHEEL_DELTA_WPARAM(wParam);
    float scrollSpeed = 5.0f;
    cameraPos += scrollSpeed * (delta > 0 ? cameraFront : -cameraFront);
    return 0;
}

      case WM_MOUSEMOVE: {
    if (wParam & MK_LBUTTON) { // only rotate when right mouse is held
        float xpos = LOWORD(lParam);
        float ypos = HIWORD(lParam);

        if (firstMouse) {
            lastX = xpos;
            lastY = ypos;
            firstMouse = false;
        }

        float xoffset = xpos - lastX;
        float yoffset = lastY - ypos;
        lastX = xpos;
        lastY = ypos;

        float sensitivity = 0.2f;
        xoffset *= sensitivity;
        yoffset *= sensitivity;

        yaw += xoffset;
        pitch += yoffset;

        if (pitch > 89.0f) pitch = 89.0f;
        if (pitch < -89.0f) pitch = -89.0f;

        glm::vec3 front;
        front.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
        front.y = sin(glm::radians(pitch));
        front.z = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
        cameraFront = glm::normalize(front);
    } else {
        firstMouse = true; // reset when button released
    }
    break;
}


        case WM_LBUTTONDOWN:
            PerformPicking(mouseX, mouseY);
            return 0;
        // other cases unchanged...

        case WM_CLOSE:
            PostQuitMessage(0);
            return 0;

        case WM_KEYDOWN:
            glm::vec3 right = glm::normalize(glm::cross(cameraFront, cameraUp));
switch (wParam) {
    case 'W': cameraPos += speed * cameraFront; break;
    case 'S': cameraPos -= speed * cameraFront; break;
    case 'A': cameraPos -= speed * right; break;
    case 'D': cameraPos += speed * right; break;
    case 'Q': cameraPos += speed * cameraUp; break;
    case 'E': cameraPos -= speed * cameraUp; break;
    case VK_UP:    cameraPos += speed * cameraFront; break;
    case VK_DOWN:  cameraPos -= speed * cameraFront; break;
    case VK_LEFT:  cameraPos -= speed * right; break;
    case VK_RIGHT: cameraPos += speed * right; break;
    

}

            return 0;  // ← move return 0 here, **after** handling all keys
    }

    return DefWindowProc(hwnd, msg, wParam, lParam);
}


int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE, LPSTR, int) {
    WNDCLASS wc = {};
    wc.style = CS_OWNDC;
    wc.lpfnWndProc = WndProc;
    wc.hInstance = hInstance;
    wc.lpszClassName = "TerrainWinClass";
    RegisterClass(&wc);

    HWND hwnd = CreateWindow("TerrainWinClass", "Terrain Viewer", WS_OVERLAPPEDWINDOW,
                             CW_USEDEFAULT, CW_USEDEFAULT, 1024, 768,
                             nullptr, nullptr, hInstance, nullptr);

    HDC hdc = GetDC(hwnd);
    PIXELFORMATDESCRIPTOR pfd = { sizeof(PIXELFORMATDESCRIPTOR), 1,
        PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER,
        PFD_TYPE_RGBA, 32, 0,0,0,0,0,0, 0,0,0,0,0, 24, 8, 0,
        PFD_MAIN_PLANE, 0, 0,0,0 };
    int pf = ChoosePixelFormat(hdc, &pfd);
    SetPixelFormat(hdc, pf, &pfd);
    HGLRC hglrc = wglCreateContext(hdc);
    wglMakeCurrent(hdc, hglrc);

    ShowWindow(hwnd, SW_SHOW);
    SetupOpenGL();
LoadUnstructuredTerrain("res/heightmap2.png", 1000);
std::cout << "Terrain points: " << terrainVertices.size() << std::endl;
cameraPos = glm::vec3(heightmapWidth / 2.0f, -heightmapHeight / 2.0f, 200.0f);
    ResizeGL(1024, 768);

    MSG msg;
    while (true) {
        if (PeekMessage(&msg, nullptr, 0, 0, PM_REMOVE)) {
            if (msg.message == WM_QUIT) break;
            TranslateMessage(&msg);
            DispatchMessage(&msg);
        } else {
            RenderScene();
            SwapBuffers(hdc);
        }
    }

    wglMakeCurrent(nullptr, nullptr);
    wglDeleteContext(hglrc);
    ReleaseDC(hwnd, hdc);
    DestroyWindow(hwnd);
    return 0;
}
