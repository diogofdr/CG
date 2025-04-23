#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cstring>
#include "../../tinyxml2/tinyxml2.h"
#include <GL/glew.h>
#include <GL/glut.h>
#endif

#define _USE_MATH_DEFINES
#include <math.h>

using namespace std;
using namespace tinyxml2;

// Global variables for window and camera
int windowWidth, windowHeight;
float cameraPosX, cameraPosY, cameraPosZ;
float cameraLookAtX, cameraLookAtY, cameraLookAtZ;
float cameraUpX, cameraUpY, cameraUpZ;
float cameraFov, cameraNear, cameraFar;

// TinyXML2 document
XMLDocument doc;


// --- Global Model Data ---
// Structure to hold model vertices.
struct Model {
    int numVertices;
    vector<float> vertices;
};

const int MAX_MODELS = 100;
Model models[MAX_MODELS];

GLuint buffers[MAX_MODELS];

// --- Scene Graph Structures ---
// Enumeration for transform types.
enum TransformType { TRANSLATE, ROTATE, SCALE, T_TRANSLATE, T_ROTATE };

struct Transform {
    TransformType   type;
    float           values[4];      // x y z  |  angle x y z
    float           time = 0.0f;  // seconds for the animation
    bool            align = false; // only for Catmull-Rom
    std::vector<float> points;      // Catmull-Rom control points (x y z …)
};


// Structure for a scene graph group.
struct Group {
    vector<Transform> transforms;   // Ordered transforms for this node.
    vector<int> modelIndices;       // Indices into the global models array.
    vector<Group*> children;        // Child groups.
};

Group* root = nullptr;              // Root of the scene graph.
vector<string> modelFiles;          // List of unique model filenames.


void normalize(float* v) {
    float l = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (l == 0) return;  v[0] /= l; v[1] /= l; v[2] /= l;
}

void cross(const float* a, const float* b, float* r) {
    r[0] = a[1] * b[2] - a[2] * b[1];
    r[1] = a[2] * b[0] - a[0] * b[2];
    r[2] = a[0] * b[1] - a[1] * b[0];
}

void multMatrixVector(const float* m, const float* v, float* r) {
    for (int i = 0;i < 4;i++)
        r[i] = v[0] * m[i * 4 + 0] + v[1] * m[i * 4 + 1] + v[2] * m[i * 4 + 2] + v[3] * m[i * 4 + 3];
}

// Catmull-Rom basis
static const float m[16] = { -0.5f, 1.5f,-1.5f, 0.5f,
                             1.0f,-2.5f, 2.0f,-0.5f,
                            -0.5f, 0.0f, 0.5f, 0.0f,
                             0.0f, 1.0f, 0.0f, 0.0f };

void getCatmullRomPoint(float t, const float* p0, const float* p1, const float* p2, const float* p3,
    float* pos, float* deriv) {
    float T[4] = { t * t * t,t * t,t,1 };
    float Td[4] = { 3 * t * t,2 * t,1,0 };

    float P[3][4];
    for (int i = 0;i < 3;i++) {
        float ctrl[4] = { p0[i],p1[i],p2[i],p3[i] };
        multMatrixVector(m, ctrl, P[i]);
    }

    for (int i = 0;i < 3;i++) {
        pos[i] = T[0] * P[i][0] + T[1] * P[i][1] + T[2] * P[i][2] + T[3] * P[i][3];
        deriv[i] = Td[0] * P[i][0] + Td[1] * P[i][1] + Td[2] * P[i][2] + Td[3] * P[i][3];
    }
}

void getGlobalCatmullRomPoint(float gt, const std::vector<float>& pts,
    float* pos, float* deriv) {
    int pointCount = pts.size() / 3;
    float t = gt * pointCount;          // where on the total curve
    int index = (int)floor(t) % pointCount;
    t = t - floor(t);                   // local segment t

    int indices[4];
    indices[0] = (index + pointCount - 1) % pointCount;
    indices[1] = index;
    indices[2] = (index + 1) % pointCount;
    indices[3] = (index + 2) % pointCount;

    const float* p0 = &pts[indices[0] * 3],
        * p1 = &pts[indices[1] * 3],
        * p2 = &pts[indices[2] * 3],
        * p3 = &pts[indices[3] * 3];

    getCatmullRomPoint(t, p0, p1, p2, p3, pos, deriv);
}

void buildRotMatrixFromDeriv(const float* deriv, float* m) {
    float Z[3] = { deriv[0],deriv[1],deriv[2] };
    normalize(Z);
    float Y[3] = { 0,1,0 };
    float X[3];
    cross(Z, Y, X); normalize(X);
    cross(X, Z, Y); normalize(Y);

    m[0] = X[0]; m[4] = X[1]; m[8] = X[2]; m[12] = 0;
    m[1] = Y[0]; m[5] = Y[1]; m[9] = Y[2]; m[13] = 0;
    m[2] = Z[0]; m[6] = Z[1]; m[10] = Z[2]; m[14] = 0;
    m[3] = 0;    m[7] = 0;    m[11] = 0;    m[15] = 1;
}


// --- XML Parsing Functions ---
// Recursively parse a <group> element.
Group* parseGroup(XMLElement* groupElem) {
    Group* group = new Group();

    // Parse optional <transform> element (if any)
    XMLElement* transformElem = groupElem->FirstChildElement("transform");
    if (transformElem) {
        for (XMLElement* t = transformElem->FirstChildElement(); t != nullptr; t = t->NextSiblingElement()) {
            string tName = t->Name();
            Transform tr;
            memset(tr.values, 0, sizeof(tr.values));    // keep it clean

            if (tName == "translate") {
                if (t->Attribute("time")) {                       // animated
                    tr.type = T_TRANSLATE;
                    t->QueryFloatAttribute("time", &tr.time);
                    tr.align = t->BoolAttribute("align");
                    for (XMLElement* p = t->FirstChildElement("point"); p; p = p->NextSiblingElement("point")) {
                        float x, y, z;
                        p->QueryFloatAttribute("x", &x);
                        p->QueryFloatAttribute("y", &y);
                        p->QueryFloatAttribute("z", &z);
                        tr.points.insert(tr.points.end(), { x, y, z });
                    }
                }
                else {                                            // static
                    tr.type = TRANSLATE;
                    t->QueryFloatAttribute("x", &tr.values[0]);
                    t->QueryFloatAttribute("y", &tr.values[1]);
                    t->QueryFloatAttribute("z", &tr.values[2]);
                }
            }
            else if (tName == "rotate") {
                if (t->Attribute("time")) {                       // animated
                    tr.type = T_ROTATE;
                    t->QueryFloatAttribute("time", &tr.time);
                    t->QueryFloatAttribute("x", &tr.values[1]);
                    t->QueryFloatAttribute("y", &tr.values[2]);
                    t->QueryFloatAttribute("z", &tr.values[3]);
                }
                else {                                            // static
                    tr.type = ROTATE;
                    t->QueryFloatAttribute("angle", &tr.values[0]);
                    t->QueryFloatAttribute("x", &tr.values[1]);
                    t->QueryFloatAttribute("y", &tr.values[2]);
                    t->QueryFloatAttribute("z", &tr.values[3]);
                }
            }
            else if (tName == "scale") {
                tr.type = SCALE;
                t->QueryFloatAttribute("x", &tr.values[0]);
                t->QueryFloatAttribute("y", &tr.values[1]);
                t->QueryFloatAttribute("z", &tr.values[2]);
            }

            group->transforms.push_back(tr);

        }
    }

    // Parse optional <models> element.
    XMLElement* modelsElem = groupElem->FirstChildElement("models");
    if (modelsElem) {
        for (XMLElement* m = modelsElem->FirstChildElement("model"); m != nullptr; m = m->NextSiblingElement("model")) {
            const char* fileAttr = m->Attribute("file");
            if (fileAttr) {
                // Check if the file is already in our list.
                int modelIndex = -1;
                for (size_t i = 0; i < modelFiles.size(); i++) {
                    if (modelFiles[i] == fileAttr) {
                        modelIndex = i;
                        break;
                    }
                }
                // If not found, add it.
                if (modelIndex == -1) {
                    modelIndex = modelFiles.size();
                    modelFiles.push_back(string(fileAttr));
                }
                group->modelIndices.push_back(modelIndex);
            }
        }
    }

    // Recursively parse child groups.
    for (XMLElement* childGroup = groupElem->FirstChildElement("group"); childGroup != nullptr; childGroup = childGroup->NextSiblingElement("group")) {
        Group* child = parseGroup(childGroup);
        group->children.push_back(child);
    }

    return group;
}

// Load the XML file and extract window, camera, and scene graph data.
void loadXMLData(const char* fname) {
    XMLError eResult = doc.LoadFile(fname);
    if (eResult != XML_SUCCESS) {
        cerr << "Erro ao abrir o arquivo XML!" << endl;
        return;
    }

    XMLElement* world = doc.FirstChildElement("world");
    if (!world) {
        cerr << "Elemento <world> não encontrado!" << endl;
        return;
    }

    // Get window dimensions.
    XMLElement* windowElem = world->FirstChildElement("window");
    if (windowElem) {
        windowElem->QueryIntAttribute("width", &windowWidth);
        windowElem->QueryIntAttribute("height", &windowHeight);
    }

    // Get camera parameters.
    XMLElement* cameraElem = world->FirstChildElement("camera");
    if (cameraElem) {
        XMLElement* position = cameraElem->FirstChildElement("position");
        if (position) {
            position->QueryFloatAttribute("x", &cameraPosX);
            position->QueryFloatAttribute("y", &cameraPosY);
            position->QueryFloatAttribute("z", &cameraPosZ);
        }
        XMLElement* lookAt = cameraElem->FirstChildElement("lookAt");
        if (lookAt) {
            lookAt->QueryFloatAttribute("x", &cameraLookAtX);
            lookAt->QueryFloatAttribute("y", &cameraLookAtY);
            lookAt->QueryFloatAttribute("z", &cameraLookAtZ);
        }
        XMLElement* up = cameraElem->FirstChildElement("up");
        if (up) {
            up->QueryFloatAttribute("x", &cameraUpX);
            up->QueryFloatAttribute("y", &cameraUpY);
            up->QueryFloatAttribute("z", &cameraUpZ);
        }
        XMLElement* projection = cameraElem->FirstChildElement("projection");
        if (projection) {
            projection->QueryFloatAttribute("fov", &cameraFov);
            projection->QueryFloatAttribute("near", &cameraNear);
            projection->QueryFloatAttribute("far", &cameraFar);
        }
    }

    // Parse the scene graph (root <group> element).
    XMLElement* groupElem = world->FirstChildElement("group");
    if (groupElem) {
        root = parseGroup(groupElem);
    }
}

// --- Model Loading and Rendering ---
// Import vertices from a model file into the global models array.
void importVertices(int modelIndex, const char* filepath) {
    if (modelIndex >= MAX_MODELS)
        return;

    ifstream file(filepath);
    if (!file.is_open()) {
        cerr << "Não foi possível abrir: " << filepath << endl;
        return;
    }

    int numV;
    file >> numV;
    models[modelIndex].numVertices = numV;
    models[modelIndex].vertices.resize(numV * 3);
    int count_points = 0;
    string line;
    getline(file, line); // consume the rest of the first line

    while (getline(file, line) && count_points < numV * 3) {
        stringstream ss(line);
        float x, y, z;
        ss >> x >> y >> z;
        models[modelIndex].vertices[count_points++] = x;
        models[modelIndex].vertices[count_points++] = y;
        models[modelIndex].vertices[count_points++] = z;
    }

    GLuint vbo;
    glGenBuffers(1, &vbo);
    buffers[modelIndex] = vbo; // salva o VBO para o modelo

    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, models[modelIndex].vertices.size() * sizeof(float), models[modelIndex].vertices.data(), GL_STATIC_DRAW);

    file.close();
}

// Render a model given its index.
void renderModel(int modelIndex) {
    Model& m = models[modelIndex];

    glBindBuffer(GL_ARRAY_BUFFER, buffers[modelIndex]);
    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(3, GL_FLOAT, 0, 0);

    glDrawArrays(GL_TRIANGLES, 0, m.numVertices);

    glDisableClientState(GL_VERTEX_ARRAY);
}


// --- Recursive Scene Graph Rendering ---
// Traverse the scene graph, applying the node's transforms, rendering its models, and then its children.
void renderGroup(Group* group) {
    glPushMatrix();

    // Apply transforms in the order they were parsed.
    for (auto& tr : group->transforms) {
        switch (tr.type) {
        case TRANSLATE:
            glTranslatef(tr.values[0], tr.values[1], tr.values[2]);
            break;
        case ROTATE:
            glRotatef(tr.values[0], tr.values[1], tr.values[2], tr.values[3]);
            break;
        case SCALE:
            glScalef(tr.values[0], tr.values[1], tr.values[2]);
            break;
        case T_TRANSLATE: {
            float elapsed = glutGet(GLUT_ELAPSED_TIME) / 1000.0f; // seconds
            float gt = fmod(elapsed, tr.time) / tr.time;
            float pos[3], deriv[3];
            getGlobalCatmullRomPoint(gt, tr.points, pos, deriv);
            glTranslatef(pos[0], pos[1], pos[2]);
            if (tr.align) {
                float m[16];
                buildRotMatrixFromDeriv(deriv, m);
                glMultMatrixf(m);
            }
            break;
        }
        case T_ROTATE: {
            float elapsed = glutGet(GLUT_ELAPSED_TIME) / 1000.0f;
            float angle = fmod(elapsed, tr.time) / tr.time * 360.0f;
            glRotatef(angle, tr.values[1], tr.values[2], tr.values[3]);
            break;
        }
        }
    }


    // Render all models attached to this group.
    for (auto modelIndex : group->modelIndices) {
        renderModel(modelIndex);
    }

    // Recursively render child groups.
    for (auto child : group->children) {
        renderGroup(child);
    }

    glPopMatrix();
}

void keys(unsigned char key, int x, int y) {
	float zoomSpeed = 0.5f; // Ajuste a velocidade de zoom conforme necessário
    float rotateSpeed = 1.0f;     // Velocidade de rotação (em graus)
    float angle = rotateSpeed * M_PI / 180.0f; // Converte para radianos

    // Vetor de direção da câmera para o ponto de observação
    float dirX = cameraLookAtX - cameraPosX;
    float dirY = cameraLookAtY - cameraPosY;
    float dirZ = cameraLookAtZ - cameraPosZ;

    // Normaliza o vetor de direção
    float length = sqrt(dirX * dirX + dirY * dirY + dirZ * dirZ);
    dirX /= length;
    dirY /= length;
    dirZ /= length;

    // Vetor "up" da câmera
    float upX = cameraUpX;
    float upY = cameraUpY;
    float upZ = cameraUpZ;

    // Vetor perpendicular à direção e ao vetor "up" (direita)
    float rightX = dirY * upZ - dirZ * upY;
    float rightY = dirZ * upX - dirX * upZ;
    float rightZ = dirX * upY - dirY * upX;

    // Normaliza o vetor perpendicular
    float rightLength = sqrt(rightX * rightX + rightY * rightY + rightZ * rightZ);
    rightX /= rightLength;
    rightY /= rightLength;
    rightZ /= rightLength;

    if (key == 'w' || key == 'W') {
        // Rotaciona para cima (em torno do eixo perpendicular à direita)
        float newDirY = dirY * cos(angle) - dirZ * sin(angle);
        float newDirZ = dirY * sin(angle) + dirZ * cos(angle);
        cameraLookAtY = cameraPosY + newDirY * length;
        cameraLookAtZ = cameraPosZ + newDirZ * length;
    }
    else if (key == 's' || key == 'S') {
        // Rotaciona para baixo (em torno do eixo perpendicular à direita)
        float newDirY = dirY * cos(-angle) - dirZ * sin(-angle);
        float newDirZ = dirY * sin(-angle) + dirZ * cos(-angle);
        cameraLookAtY = cameraPosY + newDirY * length;
        cameraLookAtZ = cameraPosZ + newDirZ * length;
    }
    else if (key == 'd' || key == 'D') {
        // Rotaciona à esquerda (em torno do eixo Y)
        float newDirX = dirX * cos(angle) - dirZ * sin(angle);
        float newDirZ = dirX * sin(angle) + dirZ * cos(angle);
        cameraLookAtX = cameraPosX + newDirX * length;
        cameraLookAtZ = cameraPosZ + newDirZ * length;
    }
    else if (key == 'a' || key == 'A') {
        // Rotaciona à direita (em torno do eixo Y)
        float newDirX = dirX * cos(-angle) - dirZ * sin(-angle);
        float newDirZ = dirX * sin(-angle) + dirZ * cos(-angle);
        cameraLookAtX = cameraPosX + newDirX * length;
        cameraLookAtZ = cameraPosZ + newDirZ * length;
    }
    else if (key == '+') {
        // Zoom In: Move a câmera para frente
        cameraPosX += dirX * zoomSpeed;
        cameraPosY += dirY * zoomSpeed;
        cameraPosZ += dirZ * zoomSpeed;
        // Atualiza o ponto de observação para manter a direção
        cameraLookAtX += dirX * zoomSpeed;
        cameraLookAtY += dirY * zoomSpeed;
        cameraLookAtZ += dirZ * zoomSpeed;
    }
    else if (key == '-') {
        // Zoom Out: Move a câmera para trás
        cameraPosX -= dirX * zoomSpeed;
        cameraPosY -= dirY * zoomSpeed;
        cameraPosZ -= dirZ * zoomSpeed;
        // Atualiza o ponto de observação para manter a direção
        cameraLookAtX -= dirX * zoomSpeed;
        cameraLookAtY -= dirY * zoomSpeed;
        cameraLookAtZ -= dirZ * zoomSpeed;
    }

    glutPostRedisplay(); // Redesenha a cena
}

void specialKeys(int key, int x, int y) {
    float moveSpeed = 0.5f; // Ajuste a velocidade de movimento conforme necessário

    // Vetor de direção da câmera para o ponto de observação
    float dirX = cameraLookAtX - cameraPosX;
    float dirY = cameraLookAtY - cameraPosY;
    float dirZ = cameraLookAtZ - cameraPosZ;

    // Normaliza o vetor de direção
    float length = sqrt(dirX * dirX + dirY * dirY + dirZ * dirZ);
    dirX /= length;
    dirY /= length;
    dirZ /= length;

    // Vetor "up" da câmera
    float upX = cameraUpX;
    float upY = cameraUpY;
    float upZ = cameraUpZ;

    // Vetor perpendicular à direção e ao vetor "up" (direita)
    float rightX = dirY * upZ - dirZ * upY;
    float rightY = dirZ * upX - dirX * upZ;
    float rightZ = dirX * upY - dirY * upX;

    // Normaliza o vetor perpendicular
    float rightLength = sqrt(rightX * rightX + rightY * rightY + rightZ * rightZ);
    rightX /= rightLength;
    rightY /= rightLength;
    rightZ /= rightLength;

    if (key == GLUT_KEY_UP) {
        // Move a câmera para cima (ao longo do vetor "up")
        cameraPosX += upX * moveSpeed;
        cameraPosY += upY * moveSpeed;
        cameraPosZ += upZ * moveSpeed;
        cameraLookAtX += upX * moveSpeed;
        cameraLookAtY += upY * moveSpeed;
        cameraLookAtZ += upZ * moveSpeed;
    }
    else if (key == GLUT_KEY_DOWN) {
        // Move a câmera para baixo (ao longo do vetor "up" negativo)
        cameraPosX -= upX * moveSpeed;
        cameraPosY -= upY * moveSpeed;
        cameraPosZ -= upZ * moveSpeed;
        cameraLookAtX -= upX * moveSpeed;
        cameraLookAtY -= upY * moveSpeed;
        cameraLookAtZ -= upZ * moveSpeed;
    }
    else if (key == GLUT_KEY_LEFT) {
        // Move a câmera para a esquerda (ao longo do vetor perpendicular negativo)
        cameraPosX -= rightX * moveSpeed;
        cameraPosY -= rightY * moveSpeed;
        cameraPosZ -= rightZ * moveSpeed;
        cameraLookAtX -= rightX * moveSpeed;
        cameraLookAtY -= rightY * moveSpeed;
        cameraLookAtZ -= rightZ * moveSpeed;
    }
    else if (key == GLUT_KEY_RIGHT) {
        // Move a câmera para a direita (ao longo do vetor perpendicular)
        cameraPosX += rightX * moveSpeed;
        cameraPosY += rightY * moveSpeed;
        cameraPosZ += rightZ * moveSpeed;
        cameraLookAtX += rightX * moveSpeed;
        cameraLookAtY += rightY * moveSpeed;
        cameraLookAtZ += rightZ * moveSpeed;
    }

    glutPostRedisplay(); // Redesenha a cena
}

// --- GLUT Callbacks ---
// Resize callback.
void changeSize(int w, int h) {
    if (h == 0)
        h = 1;
    float ratio = (float)w / (float)h;
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glViewport(0, 0, w, h);
    gluPerspective(cameraFov, ratio, cameraNear, cameraFar);
    glMatrixMode(GL_MODELVIEW);
}

// Display callback.
void renderScene(void) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    gluLookAt(cameraPosX, cameraPosY, cameraPosZ,
        cameraLookAtX, cameraLookAtY, cameraLookAtZ,
        cameraUpX, cameraUpY, cameraUpZ);

    // Render the scene graph starting from the root.
    if (root)
        renderGroup(root);

    glutSwapBuffers();
}

// --- Main Function ---
int main(int argc, char** argv) {
    loadXMLData("../../Scene/Scene.xml"); // <- aqui dentro está o glGenBuffers
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
    glutInitWindowPosition(100, 100);
    glutInitWindowSize(windowWidth, windowHeight);
    glutCreateWindow("CG@DI-UM");

#ifndef __APPLE__
    GLenum glewError = glewInit();
    if (glewError != GLEW_OK) {
        cerr << "Erro ao inicializar GLEW: " << glewGetErrorString(glewError) << endl;
        return -1;
    }
#endif

    // Só depois de GLEW + janela criada:
    for (size_t i = 0; i < modelFiles.size(); i++) {
        char path[128];
        snprintf(path, sizeof(path), "../../modelos/%s", modelFiles[i].c_str());
        importVertices(i, path);
    }

    glutKeyboardFunc(keys);
    glutSpecialFunc(specialKeys);

    glutDisplayFunc(renderScene);
    glutReshapeFunc(changeSize);
    glutIdleFunc(renderScene);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glPolygonMode(GL_FRONT, GL_LINE);
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

    glutMainLoop();
    return 0;
}
