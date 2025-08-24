#include <opencv2/opencv.hpp>
#include <GL/glut.h>
#include <iostream>

cv::Mat image;
GLuint textureID;

void loadImageWithOpenCV(const std::string& filename) {
    image = cv::imread(filename, cv::IMREAD_COLOR); // BGR by default
    if (image.empty()) {
        std::cerr << "Failed to load image: " << filename << std::endl;
        exit(1);
    }

    // Convert to RGB for OpenGL
    cv::cvtColor(image, image, cv::COLOR_BGR2RGB);

    glGenTextures(1, &textureID);
    glBindTexture(GL_TEXTURE_2D, textureID);

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,
                 image.cols, image.rows, 0,
                 GL_RGB, GL_UNSIGNED_BYTE, image.data);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
}

void display() {
    glClear(GL_COLOR_BUFFER_BIT);

    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, textureID);

    glBegin(GL_QUADS);
        glTexCoord2f(0.0f, 1.0f); glVertex2f(-1, -1);
        glTexCoord2f(1.0f, 1.0f); glVertex2f( 1, -1);
        glTexCoord2f(1.0f, 0.0f); glVertex2f( 1,  1);
        glTexCoord2f(0.0f, 0.0f); glVertex2f(-1,  1);
    glEnd();

    glDisable(GL_TEXTURE_2D);
    glutSwapBuffers();
}

void InitGL() {
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(-1, 1, -1, 1);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cout << "Usage: ./image_gl_opencv <image_path>" << std::endl;
        return 0;
    }

    glutInit(&argc, argv);
    glutInitWindowSize(800, 600);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
    glutCreateWindow("OpenCV Image in OpenGL");

    InitGL();
    loadImageWithOpenCV(argv[1]);
    glutDisplayFunc(display);

    glutMainLoop();
    return 0;
}
