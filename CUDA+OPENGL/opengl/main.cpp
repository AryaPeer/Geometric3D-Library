#include <iostream>
#include <vector>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include "Point3D.h"
#include "ConvexHull.h"

const GLuint WIDTH = 800, HEIGHT = 600;

void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mode);

int main() {
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW\n";
        return -1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow* window = glfwCreateWindow(WIDTH, HEIGHT, "Convex Hull Visualization", nullptr, nullptr);
    if (!window) {
        std::cerr << "Failed to create GLFW window\n";
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);
    glfwSetKeyCallback(window, keyCallback);

    glewExperimental = GL_TRUE;
    GLenum err = glewInit();
    if (GLEW_OK != err) {
        std::cerr << "Failed to initialize GLEW: "
                  << glewGetErrorString(err) << "\n";
        return -1;
    }

    glViewport(0, 0, WIDTH, HEIGHT);

    std::vector<Point3D<float>> points = {
        {0.0f, 0.0f, 0.0f},
        {1.0f, 0.0f, 0.2f},
        {0.5f, 1.0f, -0.2f},
        {-0.5f, 0.5f, 0.5f},
        {0.2f, -0.7f, 0.3f},
        {-0.8f, -0.2f, -0.5f},
        {0.3f, 0.3f, 0.8f}
    };

    std::vector<Point3D<float>> hullVertices;
    std::vector<std::vector<int>> hullFaces;
    ConvexHull<float>::compute(points, hullVertices, hullFaces);

    std::vector<float> vertexData;
    for (const auto& vertex : hullVertices) {
        vertexData.push_back(vertex.x);
        vertexData.push_back(vertex.y);
        vertexData.push_back(vertex.z);
    }

    std::vector<unsigned int> indexData;
    for (const auto& face : hullFaces) {
        indexData.push_back(face[0]);
        indexData.push_back(face[1]);
        indexData.push_back(face[2]);
    }

    GLuint VBO, VAO, EBO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertexData.size() * sizeof(float), vertexData.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indexData.size() * sizeof(unsigned int), indexData.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glBindVertexArray(0);

    glEnable(GL_DEPTH_TEST);

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glBindVertexArray(VAO);
        glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(indexData.size()), GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);

        glfwSwapBuffers(window);
    }

    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteBuffers(1, &EBO);

    glfwTerminate();

    return 0;
}

void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mode) {
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, GL_TRUE);
}
