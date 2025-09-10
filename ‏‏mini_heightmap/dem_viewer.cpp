#include <GL/glut.h>
#include <gdal_priv.h>
#include <iostream>
#include <cfloat>  // Include this for FLT_MAX


float* elevationData = nullptr;
int width = 0, height = 0;
float scale = 0.05f;  // vertical exaggeration

void loadDEM(const char* filename) {
    GDALAllRegister();
    GDALDataset* dataset = (GDALDataset*)GDALOpen(filename, GA_ReadOnly);
    if (!dataset) {
        std::cerr << "Failed to open DEM file." << std::endl;
        exit(1);
    }

    GDALRasterBand* band = dataset->GetRasterBand(1);
    width = band->GetXSize();
    height = band->GetYSize();

    elevationData = new float[width * height];
    CPLErr err = band->RasterIO(GF_Read, 0, 0, width, height,
                               elevationData, width, height,
                               GDT_Float32, 0, 0);
    if (err != CE_None) {
        std::cerr << "Error reading raster data." << std::endl;
        exit(1);
    }

    // Debugging: print some elevation values to verify data loading
    std::cout << "Sample elevation values: " << std::endl;
    for (int i = 0; i < 10; ++i) {
        std::cout << elevationData[i] << " ";
    }
    std::cout << std::endl;

    GDALClose(dataset);
}
void drawDEM() {
    // Find the min and max elevation values for normalization
    float minHeight = FLT_MAX, maxHeight = -FLT_MAX;
    for (int i = 0; i < width * height; ++i) {
        if (elevationData[i] < minHeight) minHeight = elevationData[i];
        if (elevationData[i] > maxHeight) maxHeight = elevationData[i];
    }

    std::cout << "Min Height: " << minHeight << ", Max Height: " << maxHeight << std::endl;

    // Normalize and apply color gradient
    for (int y = 0; y < height - 1; ++y) {
        glBegin(GL_TRIANGLE_STRIP);
        for (int x = 0; x < width; ++x) {
            float z1 = elevationData[y * width + x];
            float z2 = elevationData[(y + 1) * width + x];

            float normZ1 = (z1 - minHeight) / (maxHeight - minHeight);
            float normZ2 = (z2 - minHeight) / (maxHeight - minHeight);

            // Apply a color gradient from blue (low) to red (high)
            glColor3f(normZ1, 0.0f, 1.0f - normZ1); // Color gradient for vertex 1
            glVertex3f(x, y, z1 * 0.1f); // Apply a scaling factor

            glColor3f(normZ2, 0.0f, 1.0f - normZ2); // Color gradient for vertex 2
            glVertex3f(x, y + 1, z2 * 0.1f); // Apply a scaling factor
        }
        glEnd();
    }
}

void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    
    gluLookAt(width / 2, height / 2, 200, // Camera position
        width / 2, height / 2, 0,   // Look at the center of the terrain
        0, 1, 0);                  // Up direction

    glPushMatrix();
    drawDEM();
    glPopMatrix();

    glutSwapBuffers();
}

void initGL() {glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_DEPTH_TEST);
    
    glEnable(GL_DEPTH_TEST);
    glMatrixMode(GL_PROJECTION);
    gluPerspective(45.0, 1.0, 1.0, 1000.0);
    glMatrixMode(GL_MODELVIEW);
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: ./dem_viewer <dem_file.tif>" << std::endl;
        return 1;
    }

    loadDEM(argv[1]);

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(800, 600);
    glutCreateWindow("DEM Viewer");

    initGL();
    glutDisplayFunc(display);
    glutMainLoop();

    delete[] elevationData;
    return 0;
}
