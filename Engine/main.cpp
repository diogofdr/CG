#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <iostream>
#include <fstream>
#include <sstream>
#include "../../tinyxml2/tinyxml2.h"
#include <GL/glut.h>
#endif

#define _USE_MATH_DEFINES
#include <math.h>
#include <vector> // Para usar std::vector

using namespace tinyxml2;

tinyxml2::XMLDocument doc;

int numV;
int windowWidth, windowHeight;
float cameraPosX, cameraPosY, cameraPosZ;
float cameraLookAtX, cameraLookAtY, cameraLookAtZ;
float cameraUpX, cameraUpY, cameraUpZ;
float cameraFov, cameraNear, cameraFar;
//std::vector<const char*> modelFiles;

// Função para carregar o arquivo XML e extrair as informações
void loadXMLData(const char* fname) {
    XMLError eResult = doc.LoadFile(fname);
    if (eResult != XML_SUCCESS) {
        std::cerr << "Erro ao abrir o arquivo XML!" << std::endl;
        return;
    }

    // Extrair informações da tag <window>
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

    // Extrair informações da tag <camera>
    XMLElement* camera = world->FirstChildElement("camera");
    if (camera) {
        // Posição
        XMLElement* position = camera->FirstChildElement("position");
        if (position) {
            position->QueryFloatAttribute("x", &cameraPosX);
            position->QueryFloatAttribute("y", &cameraPosY);
            position->QueryFloatAttribute("z", &cameraPosZ);
        }

        // LookAt
        XMLElement* lookAt = camera->FirstChildElement("lookAt");
        if (lookAt) {
            lookAt->QueryFloatAttribute("x", &cameraLookAtX);
            lookAt->QueryFloatAttribute("y", &cameraLookAtY);
            lookAt->QueryFloatAttribute("z", &cameraLookAtZ);
        }

        // Up vector
        XMLElement* up = camera->FirstChildElement("up");
        if (up) {
            up->QueryFloatAttribute("x", &cameraUpX);
            up->QueryFloatAttribute("y", &cameraUpY);
            up->QueryFloatAttribute("z", &cameraUpZ);
        }

        // Projection
        XMLElement* projection = camera->FirstChildElement("projection");
        if (projection) {
            projection->QueryFloatAttribute("fov", &cameraFov);
            projection->QueryFloatAttribute("near", &cameraNear);
            projection->QueryFloatAttribute("far", &cameraFar);
        }
    }

    /*// Extrair os modelos do grupo
    XMLElement* group = world->FirstChildElement("group");
    if (group) {
        XMLElement* models = group->FirstChildElement("models");
        if (models) {
            for (XMLElement* model = models->FirstChildElement("model"); model != nullptr; model = model->NextSiblingElement("model")) {
                const char* file = model->Attribute("file");
                if (file) {
                    modelFiles.push_back(file); // Armazena o nome do arquivo 3D
                }
            }
        }
    }*/
}

/*
void importvertices(const char* dow) {

    std::ifstream file(dow);

    if (!file.is_open()) {
        std::cerr << "Não foi possível abrir o ficheiro!" << std::endl;
        return;
    }

    // Lê o número de vértices
    file >> numV;

    // Usando std::vector para gerenciar a memória de forma mais segura
    float* vert = (float*)malloc(sizeof(float) * numV * 3);

    int count_points = 0;
    std::string linha;
    std::getline(file, linha); // Para consumir a linha com o número de vértices

    // Lê os vértices
    while (std::getline(file, linha) && count_points < numV * 3) {
        std::stringstream ss(linha);
        float x, y, z;
        char virgula; // Para ler as vírgulas

        ss >> x >> virgula >> y >> virgula >> z;

        vert[count_points + 0] = x;
        vert[count_points + 1] = y;
        vert[count_points + 2] = z;

        count_points += 3;
    }
    file.close();

}*/

void changeSize(int w, int h) {
    if (h == 0) h = 1;
    float ratio = w * 1.0f / h;
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glViewport(0, 0, w, h);
    gluPerspective(cameraFov, ratio, cameraNear, cameraFar);
    glMatrixMode(GL_MODELVIEW);
}

void renderScene(void) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glLoadIdentity();
    gluLookAt(cameraPosX, cameraPosY, cameraPosZ,
        cameraLookAtX, cameraLookAtY, cameraLookAtZ,
        cameraUpX, cameraUpY, cameraUpZ);

    glutWireTeapot(1);

    glutSwapBuffers();
}

int main(int argc, char** argv) {
    loadXMLData("../../Scene/Scene.xml");
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
