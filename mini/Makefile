# Output program
TARGET = read_Image_exec

# Source files
SRCS = readImage.cpp

# Compiler
CXX = g++
CXXFLAGS = -Wall -g

# OpenCV flags
OPENCV_CFLAGS = $(shell pkg-config --cflags opencv4)
OPENCV_LIBS   = $(shell pkg-config --libs opencv4)

# Final build rule
all: $(TARGET)

$(TARGET): $(SRCS)
	$(CXX) $(CXXFLAGS) $(OPENCV_CFLAGS) -o $@ $^ $(OPENCV_LIBS) -lGL -lGLU -lglut

clean:
	rm -f $(TARGET)
