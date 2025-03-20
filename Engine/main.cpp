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

// --- Scene Graph Structures ---
// Enumeration for transform types.
enum TransformType { TRANSLATE, ROTATE, SCALE };

// Structure to store a transform in the order it is defined.
struct Transform {
    TransformType type;
    // For translate and scale: values[0..2] (x, y, z).
    // For rotate: values[0] = angle, values[1..3] = axis (x, y, z).
    float values[4];
};

// Structure for a scene graph group.
struct Group {
    vector<Transform> transforms;   // Ordered transforms for this node.
    vector<int> modelIndices;       // Indices into the global models array.
    vector<Group*> children;        // Child groups.
};

Group* root = nullptr;              // Root of the scene graph.
vector<string> modelFiles;          // List of unique model filenames.

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
            if (tName == "translate") {
                tr.type = TRANSLATE;
                t->QueryFloatAttribute("x", &tr.values[0]);
                t->QueryFloatAttribute("y", &tr.values[1]);
                t->QueryFloatAttribute("z", &tr.values[2]);
            }
            else if (tName == "rotate") {
                tr.type = ROTATE;
                t->QueryFloatAttribute("angle", &tr.values[0]);
                t->QueryFloatAttribute("x", &tr.values[1]);
                t->QueryFloatAttribute("y", &tr.values[2]);
                t->QueryFloatAttribute("z", &tr.values[3]);
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
    file.close();
}

// Render a model given its index.
void renderModel(int modelIndex) {
    // Ensure the model index is valid.
    Model& m = models[modelIndex];
    int totalCoords = m.numVertices * 3;
    // We assume that each triangle is composed of 3 vertices (9 floats).
    // Loop in steps of 9.
    for (int i = 0; i < totalCoords; i += 9) {
        glColor3f(1.0f, 1.0f, 1.0f);
        glBegin(GL_TRIANGLES);
        glVertex3f(m.vertices[i + 0], m.vertices[i + 1], m.vertices[i + 2]);
        glVertex3f(m.vertices[i + 3], m.vertices[i + 4], m.vertices[i + 5]);
        glVertex3f(m.vertices[i + 6], m.vertices[i + 7], m.vertices[i + 8]);
        glEnd();
    }
}

// --- Recursive Scene Graph Rendering ---
// Traverse the scene graph, applying the node's transforms, rendering its models, and then its children.
void renderGroup(Group* group) {
    glPushMatrix();

    // Apply transforms in the order they were parsed.
    for (auto& tr : group->transforms) {
        if (tr.type == TRANSLATE) {
            glTranslatef(tr.values[0], tr.values[1], tr.values[2]);
        }
        else if (tr.type == ROTATE) {
            glRotatef(tr.values[0], tr.values[1], tr.values[2], tr.values[3]);
        }
        else if (tr.type == SCALE) {
            glScalef(tr.values[0], tr.values[1], tr.values[2]);
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
    // Load the XML scene (adjust the path as needed).
    loadXMLData("../../Scene/Scene.xml");

    // For every unique model file, load the vertices.
    for (size_t i = 0; i < modelFiles.size(); i++) {
        char path[128];
        snprintf(path, sizeof(path), "../../modelos/%s", modelFiles[i].c_str());
        importVertices(i, path);
    }

    // Initialize GLUT.
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
    return 0;
}
