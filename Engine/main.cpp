#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <iostream>
#include <fstream>
#include <sstream>
#include "../../tinyxml2/tinyxml2.h"
#include <../glew/GL/glew.h>
#include <GL/glut.h>
#endif

#define _USE_MATH_DEFINES
#include <math.h>
#include <vector> // Para usar std::vector

using namespace tinyxml2;

#define MAX_MODELS 100
#define MAX_FILENAME_LENGTH 64

tinyxml2::XMLDocument doc;

int windowWidth, windowHeight;
float cameraPosX, cameraPosY, cameraPosZ;
float cameraLookAtX, cameraLookAtY, cameraLookAtZ;
float cameraUpX, cameraUpY, cameraUpZ;
float cameraFov, cameraNear, cameraFar;

char modelfiles[MAX_MODELS][MAX_FILENAME_LENGTH];
int modelCount = 0;

// Estrutura para guardar os vértices de cada modelo
struct Model {
    int numVertices;
    std::vector<float> vertices;
};

Model models[MAX_MODELS]; // Array com os dados reais

void loadXMLData(const char* fname) {
    XMLError eResult = doc.LoadFile(fname);
    if (eResult != XML_SUCCESS) {
        std::cerr << "Erro ao abrir o arquivo XML!" << std::endl;
        return;
    }

    XMLElement* world = doc.FirstChildElement("world");
    if (!world) {
        std::cerr << "Elemento <world> não encontrado!" << std::endl;
        return;
    }

    XMLElement* window = world->FirstChildElement("window");
    if (window) {
        window->QueryIntAttribute("width", &windowWidth);
        window->QueryIntAttribute("height", &windowHeight);
    }

    XMLElement* camera = world->FirstChildElement("camera");
    if (camera) {
        XMLElement* position = camera->FirstChildElement("position");
        if (position) {
            position->QueryFloatAttribute("x", &cameraPosX);
            position->QueryFloatAttribute("y", &cameraPosY);
            position->QueryFloatAttribute("z", &cameraPosZ);
        }

        XMLElement* lookAt = camera->FirstChildElement("lookAt");
        if (lookAt) {
            lookAt->QueryFloatAttribute("x", &cameraLookAtX);
            lookAt->QueryFloatAttribute("y", &cameraLookAtY);
            lookAt->QueryFloatAttribute("z", &cameraLookAtZ);
        }

        XMLElement* up = camera->FirstChildElement("up");
        if (up) {
            up->QueryFloatAttribute("x", &cameraUpX);
            up->QueryFloatAttribute("y", &cameraUpY);
            up->QueryFloatAttribute("z", &cameraUpZ);
        }

        XMLElement* projection = camera->FirstChildElement("projection");
        if (projection) {
            projection->QueryFloatAttribute("fov", &cameraFov);
            projection->QueryFloatAttribute("near", &cameraNear);
            projection->QueryFloatAttribute("far", &cameraFar);
        }
    }

    XMLElement* group = world->FirstChildElement("group");
    if (group) {
        XMLElement* modelsNode = group->FirstChildElement("models");
        if (modelsNode) {
            XMLElement* model = modelsNode->FirstChildElement("model");
            while (model && modelCount < MAX_MODELS) {
                const char* fileName = model->Attribute("file");
                if (fileName) {
                    strncpy(modelfiles[modelCount], fileName, MAX_FILENAME_LENGTH - 1);
                    modelfiles[modelCount][MAX_FILENAME_LENGTH - 1] = '\0';
                    modelCount++;
                }
                model = model->NextSiblingElement("model");
            }
        }
    }
}

// Função que importa vértices para um modelo específico
void importvertices(int modelIndex, const char* filepath) {
    std::ifstream file(filepath);

    if (!file.is_open()) {
        std::cerr << "Não foi possível abrir: " << filepath << std::endl;
        return;
    }

    int numV;
    file >> numV;
    models[modelIndex].numVertices = numV;
    models[modelIndex].vertices.resize(numV * 3);

    int count_points = 0;
    std::string linha;
    std::getline(file, linha); // Consome a linha do numV

    while(std::getline(file, linha) && count_points < numV * 3) {
        std::stringstream ss(linha);
        float x, y, z;

        ss >> x >> y >> z;  // Simples espaço separado

        models[modelIndex].vertices[count_points + 0] = x;
        models[modelIndex].vertices[count_points + 1] = y;
        models[modelIndex].vertices[count_points + 2] = z;

        printf("Vertex %d: %f %f %f\n", count_points / 3, x, y, z); // Para debug

        count_points += 3;
    }

    file.close();
}

void changeSize(int w, int h) {
    if (h == 0) h = 1;
    float ratio = w * 1.0f / h;
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glViewport(0, 0, w, h);
    gluPerspective(cameraFov, ratio, cameraNear, cameraFar);
    glMatrixMode(GL_MODELVIEW);
}

// Desenha cada modelo
void renderScene(void) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    gluLookAt(cameraPosX, cameraPosY, cameraPosZ,
        cameraLookAtX, cameraLookAtY, cameraLookAtZ,
        cameraUpX, cameraUpY, cameraUpZ);

    /*// Desenho dos eixos
    glBegin(GL_LINES);
    glColor3f(1.0f, 0.0f, 0.0f); // Eixo X
    glVertex3f(-100.0f, 0.0f, 0.0f);
    glVertex3f(100.0f, 0.0f, 0.0f);
    glColor3f(0.0f, 1.0f, 0.0f); // Eixo Y
    glVertex3f(0.0f, -100.0f, 0.0f);
    glVertex3f(0.0f, 100.0f, 0.0f);
    glColor3f(0.0f, 0.0f, 1.0f); // Eixo Z
    glVertex3f(0.0f, 0.0f, -100.0f);
    glVertex3f(0.0f, 0.0f, 100.0f);
    glEnd();*/


    for (int i = 0; i < modelCount; i++) {
        for (int v = 0; v < models[i].numVertices; v+=3) {
            glColor3f(1.0f, 1.0f, 1.0f);
            glBegin(GL_TRIANGLES);
            glVertex3f(models[i].vertices[v * 3 + 0],
                models[i].vertices[v * 3 + 1],
                models[i].vertices[v * 3 + 2]);
            glVertex3f(models[i].vertices[(v + 1) * 3 + 0],
                models[i].vertices[(v + 1) * 3 + 1],
                models[i].vertices[(v + 1) * 3 + 2]);
            glVertex3f(models[i].vertices[(v + 2) * 3 + 0],
                models[i].vertices[(v + 2) * 3 + 1],
                models[i].vertices[(v + 2) * 3 + 2]);
            glEnd();
        }
    }

    glutSwapBuffers();
}

int main(int argc, char** argv) {
    loadXMLData("../../Scene/Scene.xml");
    /*
    char path[128];
    snprintf(path, sizeof(path), "../../modelos/output_%s", modelfiles[2]);
    importvertices(1, path);*/

    // Carregar os vértices de cada modelo
    for (int i = 0; i < modelCount; i++) {
        char path[128];
        snprintf(path, sizeof(path), "../../modelos/%s", modelfiles[i]);
        importvertices(i, path);
    }

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
    glutInitWindowPosition(100, 100);
    glutInitWindowSize(windowWidth, windowHeight);
    glutCreateWindow("CG@DI");

    glutDisplayFunc(renderScene);
    glutReshapeFunc(changeSize);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glPolygonMode(GL_FRONT, GL_LINE);
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

    glutMainLoop();
    return 1;
}
