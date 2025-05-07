/*
 * Solar-System Engine – Phase 3 (Refactored)
 * ------------------------------------------------------------------------
 *  • VBO-only rendering for maximum throughput (no immediate mode leftovers)
 *  • Static + time-based transforms (Catmull-Rom translate, timed rotate)
 *  • Scene graph parsed from XML (tinyxml2)
 *  • Keyboard navigation: WASD + / - for zoom, arrow keys for strafing
 *
 *  Build example (Linux):
 *      g++ SolarSystemRefactored.cpp tinyxml2.cpp -lGL -lGLU -lglut -lGLEW -std=c++17
 *
 */

 // -------------------------------------------------------------------------
 // 1.  INCLUDES & PLATFORM QUIRKS
 // -------------------------------------------------------------------------
#ifdef _WIN32
#define GLUT_DISABLE_ATEXIT_HACK  // avoid C2381: exit redefinition (GLUT bug)
#endif

#define _USE_MATH_DEFINES         // enables M_PI in <cmath> on MSVC
#include <cmath>
#ifndef M_PI                       // fallback for exotic compilers
#define M_PI 3.14159265358979323846
#endif

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glew.h>
#include <GL/glut.h>
#endif

#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include "../../tinyxml2/tinyxml2.h"
#include <IL/il.h>

using namespace std;
using namespace tinyxml2;

// -------------------------------------------------------------------------
// 2.  GLOBAL WINDOW & CAMERA STATE (keep minimal – refactor further if bored)
// -------------------------------------------------------------------------
int   g_windowWidth = 800;
int   g_windowHeight = 600;

float g_camPos[3] = { 0.f, 0.f, 20.f };
float g_camLook[3] = { 0.f, 0.f, 0.f };
float g_camUp[3] = { 0.f, 1.f, 0.f };

float g_camFov = 60.f;
float g_camNear = 1.f;
float g_camFar = 500.f;

XMLDocument g_doc;   // lifetime = entire program

// -------------------------------------------------------------------------
// 3.  GPU MODEL STORAGE (VBOs only – no client-side arrays at draw time)
// -------------------------------------------------------------------------
struct Model {
    int           vertexCount = 0;       // number of vertices (triangle vertices, not faces)
    vector<float> vertices;              // tightly-packed xyz
    GLuint        vbo = 0;               // OpenGL buffer object id
    int           normalCount = 0;       // number of normals
    vector<float> normals;               // tightly-packed xyz
    GLuint        nom = 0;               // OpenGL buffer object id
    int           textCount = 0;         // number of UV (points on texture)
    vector<float> texCoord;              // tightly-packed xy
    GLuint        tex = 2;               // OpenGL buffer object id
    GLuint texId = 0;
};

constexpr int kMaxModels = 128;          // raise if your scene explodes
vector<Model>       g_models;            // dynamic to track real usage
vector<string>      g_modelFiles;        // unique model filenames, index matches g_models
vector<string>      g_textureFiles;      // unique texture filenames, index matches g_models

// -------------------------------------------------------------------------
// 4.  SCENE GRAPH NODES & TRANSFORMS
// -------------------------------------------------------------------------
enum class TransformType {
    TranslateStatic,
    RotateStatic,
    ScaleStatic,
    TranslateTimed,
    RotateTimed
};

struct Transform {
    TransformType  type;
    float          data[4] = { 0.f };  // translate/scale: xyz | rotate: angle xyz
    float          duration = 0.f;    // seconds for a full animation loop
    bool           align = false;  // align tangent for Catmull-Rom paths
    vector<float>  control;            // control points (Catmull-Rom): x y z …
};

struct Group {
    vector<Transform> transforms;   // ordered list applied to this node
    vector<int>       modelIdx;     // indices into g_models
    vector<unique_ptr<Group>> children; // child nodes – smart pointers avoid leaks
};

unique_ptr<Group> g_root;           // the entire scene

// -------------------------------------------------------------------------
// 5.  MATH HELPERS (Catmull-Rom & basic linear algebra) – NO GLM to keep
//     the project dependency-free.
// -------------------------------------------------------------------------
namespace math {

    inline void normalize(float* v) {
        const float len = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
        if (len == 0.f) return;
        v[0] /= len; v[1] /= len; v[2] /= len;
    }

    inline void cross(const float* a, const float* b, float* r) {
        r[0] = a[1] * b[2] - a[2] * b[1];
        r[1] = a[2] * b[0] - a[0] * b[2];
        r[2] = a[0] * b[1] - a[1] * b[0];
    }

    inline void multMatVec(const float* m, const float* v, float* r) {
        for (int i = 0; i < 4; ++i)
            r[i] = v[0] * m[i * 4 + 0] + v[1] * m[i * 4 + 1] + v[2] * m[i * 4 + 2] + v[3] * m[i * 4 + 3];
    }

    // Catmull-Rom basis matrix in column-major form
    constexpr float kCatmull[16] = {
        -0.5f,  1.5f, -1.5f,  0.5f,
         1.0f, -2.5f,  2.0f, -0.5f,
        -0.5f,  0.0f,  0.5f,  0.0f,
         0.0f,  1.0f,  0.0f,  0.0f
    };

    void getCatmullRomPoint(float t, const float* p0, const float* p1, const float* p2, const float* p3,
        float* pos, float* deriv) {
        const float T[4] = { t * t * t, t * t, t, 1.f };
        const float Td[4] = { 3 * t * t, 2 * t, 1.f, 0.f };

        float A[3][4];
        for (int i = 0; i < 3; ++i) {
            float ctrl[4] = { p0[i], p1[i], p2[i], p3[i] };
            multMatVec(kCatmull, ctrl, A[i]);
        }
        for (int i = 0; i < 3; ++i) {
            pos[i] = T[0] * A[i][0] + T[1] * A[i][1] + T[2] * A[i][2] + T[3] * A[i][3];
            deriv[i] = Td[0] * A[i][0] + Td[1] * A[i][1] + Td[2] * A[i][2] + Td[3] * A[i][3];
        }
    }

    void getGlobalCatmullRom(float gt, const vector<float>& pts, float* pos, float* deriv) {
        const int count = static_cast<int>(pts.size() / 3);
        const float t = gt * count;
        const int seg = static_cast<int>(floorf(t)) % count;
        const float localT = t - floorf(t);

        const int idx[4] = { (seg + count - 1) % count, seg, (seg + 1) % count, (seg + 2) % count };

        const float* p0 = &pts[idx[0] * 3];
        const float* p1 = &pts[idx[1] * 3];
        const float* p2 = &pts[idx[2] * 3];
        const float* p3 = &pts[idx[3] * 3];

        getCatmullRomPoint(localT, p0, p1, p2, p3, pos, deriv);
    }

    void buildAlignMatrix(const float* deriv, float* out) {
        float Z[3] = { deriv[0], deriv[1], deriv[2] };
        normalize(Z);

        const float up[3] = { 0.f, 1.f, 0.f };
        float X[3];
        cross(Z, up, X);
        normalize(X);

        float Y[3];
        cross(X, Z, Y);
        normalize(Y);

        // Column-major matrix (OpenGL default)
        out[0] = X[0]; out[4] = X[1]; out[8] = X[2]; out[12] = 0.f;
        out[1] = Y[0]; out[5] = Y[1]; out[9] = Y[2]; out[13] = 0.f;
        out[2] = Z[0]; out[6] = Z[1]; out[10] = Z[2]; out[14] = 0.f;
        out[3] = 0.f;  out[7] = 0.f;  out[11] = 0.f; out[15] = 1.f;
    }

} // namespace math

// -------------------------------------------------------------------------
// 6.  XML PARSING → SCENE GRAPH (recursive descent)
// -------------------------------------------------------------------------
static unique_ptr<Group> parseGroup(XMLElement* gElem) {
    auto group = make_unique<Group>();

    // -------- transforms ------------------------------------------------
    if (auto* tElem = gElem->FirstChildElement("transform")) {
        for (auto* t = tElem->FirstChildElement(); t; t = t->NextSiblingElement()) {
            string tag = t->Name();
            Transform tr{};  // zero-init

            // TRANSLATE ---------------------------------------------------
            if (tag == "translate") {
                if (t->Attribute("time")) {    // timed path
                    tr.type = TransformType::TranslateTimed;
                    t->QueryFloatAttribute("time", &tr.duration);
                    tr.align = t->BoolAttribute("align");
                    for (auto* p = t->FirstChildElement("point"); p; p = p->NextSiblingElement("point")) {
                        float x = 0, y = 0, z = 0;
                        p->QueryFloatAttribute("x", &x);
                        p->QueryFloatAttribute("y", &y);
                        p->QueryFloatAttribute("z", &z);
                        tr.control.insert(tr.control.end(), { x, y, z });
                    }
                }
                else {                       // static translate
                    tr.type = TransformType::TranslateStatic;
                    t->QueryFloatAttribute("x", &tr.data[0]);
                    t->QueryFloatAttribute("y", &tr.data[1]);
                    t->QueryFloatAttribute("z", &tr.data[2]);
                }
            }
            // ROTATE ------------------------------------------------------
            else if (tag == "rotate") {
                if (t->Attribute("time")) {
                    tr.type = TransformType::RotateTimed;
                    t->QueryFloatAttribute("time", &tr.duration);
                    t->QueryFloatAttribute("x", &tr.data[1]);
                    t->QueryFloatAttribute("y", &tr.data[2]);
                    t->QueryFloatAttribute("z", &tr.data[3]);
                }
                else {
                    tr.type = TransformType::RotateStatic;
                    t->QueryFloatAttribute("angle", &tr.data[0]);
                    t->QueryFloatAttribute("x", &tr.data[1]);
                    t->QueryFloatAttribute("y", &tr.data[2]);
                    t->QueryFloatAttribute("z", &tr.data[3]);
                }
            }
            // SCALE -------------------------------------------------------
            else if (tag == "scale") {
                tr.type = TransformType::ScaleStatic;
                t->QueryFloatAttribute("x", &tr.data[0]);
                t->QueryFloatAttribute("y", &tr.data[1]);
                t->QueryFloatAttribute("z", &tr.data[2]);
            }

            group->transforms.push_back(std::move(tr));
        }
    }

    // -------- models ----------------------------------------------------
    if (auto* modelsElem = gElem->FirstChildElement("models")) {
        for (auto* m = modelsElem->FirstChildElement("model"); m; m = m->NextSiblingElement("model")) {
            const char* file = m->Attribute("file");
            if (!file) continue;

            // reuse VBO if already loaded
            auto it = find(g_modelFiles.begin(), g_modelFiles.end(), file);
            int idx = 0;
            if (it == g_modelFiles.end()) {
                idx = static_cast<int>(g_modelFiles.size());
                g_modelFiles.push_back(file);
                g_models.emplace_back();            // allocate slot
            }
            else {
                idx = static_cast<int>(distance(g_modelFiles.begin(), it));
            }
            group->modelIdx.push_back(idx);
        }
    }

    // -------- children --------------------------------------------------
    for (auto* child = gElem->FirstChildElement("group"); child; child = child->NextSiblingElement("group"))
        group->children.push_back(parseGroup(child));

    return group;
}

static void loadXML(const char* filename) {
    if (g_doc.LoadFile(filename) != XML_SUCCESS) {
        cerr << "[FATAL] Cannot open XML: " << filename << '\n';
        exit(EXIT_FAILURE);
    }

    auto* world = g_doc.FirstChildElement("world");
    if (!world) {
        cerr << "[FATAL] <world> element missing" << '\n';
        exit(EXIT_FAILURE);
    }

    // window -------------------------------------------------------------
    if (auto* win = world->FirstChildElement("window")) {
        win->QueryIntAttribute("width", &g_windowWidth);
        win->QueryIntAttribute("height", &g_windowHeight);
    }

    // camera -------------------------------------------------------------
    if (auto* cam = world->FirstChildElement("camera")) {
        if (auto* pos = cam->FirstChildElement("position")) {
            pos->QueryFloatAttribute("x", &g_camPos[0]);
            pos->QueryFloatAttribute("y", &g_camPos[1]);
            pos->QueryFloatAttribute("z", &g_camPos[2]);
        }
        if (auto* look = cam->FirstChildElement("lookAt")) {
            look->QueryFloatAttribute("x", &g_camLook[0]);
            look->QueryFloatAttribute("y", &g_camLook[1]);
            look->QueryFloatAttribute("z", &g_camLook[2]);
        }
        if (auto* up = cam->FirstChildElement("up")) {
            up->QueryFloatAttribute("x", &g_camUp[0]);
            up->QueryFloatAttribute("y", &g_camUp[1]);
            up->QueryFloatAttribute("z", &g_camUp[2]);
        }
        if (auto* proj = cam->FirstChildElement("projection")) {
            proj->QueryFloatAttribute("fov", &g_camFov);
            proj->QueryFloatAttribute("near", &g_camNear);
            proj->QueryFloatAttribute("far", &g_camFar);
        }
    }

    // root group ---------------------------------------------------------
    g_root = parseGroup(world->FirstChildElement("group"));
}

// -------------------------------------------------------------------------
// 7.  MODEL LOADING (simple text dump: first int = vertex count, then xyz...) 
// -------------------------------------------------------------------------

int loadTexture(std::string s) {
    unsigned int t, tw, th;
    unsigned char* texData;
    unsigned int texID;

    ilInit();
    ilEnable(IL_ORIGIN_SET);
    ilOriginFunc(IL_ORIGIN_LOWER_LEFT);
    ilGenImages(1, &t);
    ilBindImage(t);
    ilLoadImage((ILstring)s.c_str());
    if (!ilLoadImage((ILstring)s.c_str())) {
        return 0;
    }

    tw = ilGetInteger(IL_IMAGE_WIDTH);
    th = ilGetInteger(IL_IMAGE_HEIGHT);
    ilConvertImage(IL_RGBA, IL_UNSIGNED_BYTE);
    texData = ilGetData();

    glGenTextures(1, &texID);

    glBindTexture(GL_TEXTURE_2D, texID);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, tw, th, 0, GL_RGBA, GL_UNSIGNED_BYTE, texData);
    glGenerateMipmap(GL_TEXTURE_2D);

    glBindTexture(GL_TEXTURE_2D, 0);

    return texID;

}
static void loadModelVertices(int modelIndex, const char* path, const char* patht) {
    if (modelIndex >= kMaxModels) {
        cerr << "[WARN] Model index overflow – raise kMaxModels" << '\n';
        return;
    }

    ifstream file(path);
    if (!file) {
        cerr << "[ERROR] Cannot open model file: " << path << '\n';
        return;
    }

    GLuint texId = loadTexture(patht);
    if (texId == 0) {
        std::cerr << "Failed to load texture!\n";
    }
    else {
        g_models[modelIndex].texId = texId;
    }

    int n = 0;
    file >> n;
    if (n <= 0) {
        cerr << "[ERROR] Invalid vertex count in: " << path << '\n';
        return;
    }

    auto& model = g_models[modelIndex];
    model.vertexCount = n;
    model.textCount = n;
    model.vertices.resize(n * 3);
    model.normals.resize(n * 3);
    model.texCoord.resize(n * 2);

    for (int i = 0; i < n * 3; ++i) file >> model.vertices[i];

    for (int i = 0; i < n; ++i) {
        float x = model.vertices[i * 3 + 0];
        float y = model.vertices[i * 3 + 1];
        float z = model.vertices[i * 3 + 2];
        float len = sqrtf(x * x + y * y + z * z);

        if (len > 0.0001f) {
            x /= len; y /= len; z /= len;
        }

        model.normals[i * 3 + 0] = x;
        model.normals[i * 3 + 1] = y;
        model.normals[i * 3 + 2] = z;

        model.texCoord[i * 2 + 0] = 0.5f + atan2f(z, x) / (2.0f * M_PI);
        model.texCoord[i * 2 + 1] = 0.5f + asinf(y) / M_PI;
    }

    // VBOs
    glGenBuffers(1, &model.vbo);
    glBindBuffer(GL_ARRAY_BUFFER, model.vbo);
    glBufferData(GL_ARRAY_BUFFER, model.vertices.size() * sizeof(float), model.vertices.data(), GL_STATIC_DRAW);

    glGenBuffers(1, &model.nom);
    glBindBuffer(GL_ARRAY_BUFFER, model.nom);
    glBufferData(GL_ARRAY_BUFFER, model.normals.size() * sizeof(float), model.normals.data(), GL_STATIC_DRAW);

    glGenBuffers(1, &model.tex);
    glBindBuffer(GL_ARRAY_BUFFER, model.tex);
    glBufferData(GL_ARRAY_BUFFER, model.texCoord.size() * sizeof(float), model.texCoord.data(), GL_STATIC_DRAW);
}


void renderModel(int idx) {
    const Model& m = g_models[idx];

    float white[4] = { 1.0f, 1.0f, 1.0f, 1.0f };
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, white);

    if (m.texId != 0) {
        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, m.texId);
    }

    glBindBuffer(GL_ARRAY_BUFFER, m.vbo);
    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(3, GL_FLOAT, 0, nullptr);

    glBindBuffer(GL_ARRAY_BUFFER, m.nom);
    glEnableClientState(GL_NORMAL_ARRAY);
    glNormalPointer(GL_FLOAT, 0, nullptr);

    glBindBuffer(GL_ARRAY_BUFFER, m.tex);
    glEnableClientState(GL_TEXTURE_COORD_ARRAY);
    glTexCoordPointer(2, GL_FLOAT, 0, nullptr);

    glDrawArrays(GL_TRIANGLES, 0, m.vertexCount);

    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_NORMAL_ARRAY);
    glDisableClientState(GL_TEXTURE_COORD_ARRAY);

    if (m.texId != 0) {
        glBindTexture(GL_TEXTURE_2D, 0);
        glDisable(GL_TEXTURE_2D);
    }
}



// -------------------------------------------------------------------------
// 8.  RENDERING (recursive descent of the scene graph)
// -------------------------------------------------------------------------
static void renderGroup(const Group* g) {
    glPushMatrix();

    // apply transforms in declared order --------------------------------
    for (const auto& tr : g->transforms) {
        switch (tr.type) {
        case TransformType::TranslateStatic:
            glTranslatef(tr.data[0], tr.data[1], tr.data[2]);
            break;

        case TransformType::RotateStatic:
            glRotatef(tr.data[0], tr.data[1], tr.data[2], tr.data[3]);
            break;

        case TransformType::ScaleStatic:
            glScalef(tr.data[0], tr.data[1], tr.data[2]);
            break;

        case TransformType::TranslateTimed: {
            const float elapsed = glutGet(GLUT_ELAPSED_TIME) / 1000.f;
            const float gt = fmod(elapsed, tr.duration) / tr.duration;
            float pos[3], deriv[3];
            math::getGlobalCatmullRom(gt, tr.control, pos, deriv);
            glTranslatef(pos[0], pos[1], pos[2]);
            if (tr.align) {
                float m[16];
                math::buildAlignMatrix(deriv, m);
                glMultMatrixf(m);
            }
        } break;

        case TransformType::RotateTimed: {
            const float elapsed = glutGet(GLUT_ELAPSED_TIME) / 1000.f;
            const float angle = fmod(elapsed, tr.duration) / tr.duration * 360.f;
            glRotatef(angle, tr.data[1], tr.data[2], tr.data[3]);
        } break;
        }
    }

    // draw all models attached to this node ------------------------------
    for (int idx : g->modelIdx) renderModel(idx);

    // recurse into children ---------------------------------------------
    for (const auto& child : g->children) renderGroup(child.get());

    glPopMatrix();
}

// -------------------------------------------------------------------------
// 9.  INPUT – bare-bones camera manipulation (no mouse, no smoothing)
// -------------------------------------------------------------------------
static void keyboard(unsigned char key, int, int) {
    constexpr float zoomStep = 0.5f;
    constexpr float rotStep = M_PI / 180.f; // 1 degree

    // view direction -----------------------------------------------------
    float dir[3] = { g_camLook[0] - g_camPos[0],
                    g_camLook[1] - g_camPos[1],
                    g_camLook[2] - g_camPos[2] };
    math::normalize(dir);

    // right vector -------------------------------------------------------
    float right[3];
    math::cross(dir, g_camUp, right);
    math::normalize(right);

    switch (key) {
    case 'w': case 'W': // pitch up
        g_camLook[1] += cosf(rotStep) * dir[1] - sinf(rotStep) * dir[2];
        g_camLook[2] += sinf(rotStep) * dir[1] + cosf(rotStep) * dir[2];
        break;
    case 's': case 'S': // pitch down
        g_camLook[1] += cosf(-rotStep) * dir[1] - sinf(-rotStep) * dir[2];
        g_camLook[2] += sinf(-rotStep) * dir[1] + cosf(-rotStep) * dir[2];
        break;
    case 'd': case 'D': // yaw left
        g_camLook[0] += cosf(rotStep) * dir[0] - sinf(rotStep) * dir[2];
        g_camLook[2] += sinf(rotStep) * dir[0] + cosf(rotStep) * dir[2];
        break;
    case 'a': case 'A': // yaw right
        g_camLook[0] += cosf(-rotStep) * dir[0] - sinf(-rotStep) * dir[2];
        g_camLook[2] += sinf(-rotStep) * dir[0] + cosf(-rotStep) * dir[2];
        break;
    case '+': // zoom in
        for (int i = 0; i < 3; ++i) {
            g_camPos[i] += dir[i] * zoomStep;
            g_camLook[i] += dir[i] * zoomStep;
        }
        break;
    case '-': // zoom out
        for (int i = 0; i < 3; ++i) {
            g_camPos[i] -= dir[i] * zoomStep;
            g_camLook[i] -= dir[i] * zoomStep;
        }
        break;
    }
    glutPostRedisplay();
}

static void special(int key, int, int) {
    constexpr float move = 0.5f;

    float dir[3] = { g_camLook[0] - g_camPos[0],
                    g_camLook[1] - g_camPos[1],
                    g_camLook[2] - g_camPos[2] };
    math::normalize(dir);

    float right[3];
    math::cross(dir, g_camUp, right);
    math::normalize(right);

    switch (key) {
    case GLUT_KEY_UP:   // move up
        for (int i = 0; i < 3; ++i) {
            g_camPos[i] += g_camUp[i] * move;
            g_camLook[i] += g_camUp[i] * move;
        }
        break;
    case GLUT_KEY_DOWN: // move down
        for (int i = 0; i < 3; ++i) {
            g_camPos[i] -= g_camUp[i] * move;
            g_camLook[i] -= g_camUp[i] * move;
        }
        break;
    case GLUT_KEY_LEFT: // strafe left
        for (int i = 0; i < 3; ++i) {
            g_camPos[i] -= right[i] * move;
            g_camLook[i] -= right[i] * move;
        }
        break;
    case GLUT_KEY_RIGHT: // strafe right
        for (int i = 0; i < 3; ++i) {
            g_camPos[i] += right[i] * move;
            g_camLook[i] += right[i] * move;
        }
        break;
    }
    glutPostRedisplay();
}

// -------------------------------------------------------------------------
// 10. GLUT CALLBACKS
// -------------------------------------------------------------------------
static void reshape(int w, int h) {
    if (h == 0) h = 1; // avoid divide-by-zero
    const float ratio = static_cast<float>(w) / h;

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(g_camFov, ratio, g_camNear, g_camFar);
    glViewport(0, 0, w, h);
    glMatrixMode(GL_MODELVIEW);
}

static void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    gluLookAt(g_camPos[0], g_camPos[1], g_camPos[2],
        g_camLook[0], g_camLook[1], g_camLook[2],
        g_camUp[0], g_camUp[1], g_camUp[2]);

    if (g_root) renderGroup(g_root.get());

    glutSwapBuffers();
}

// -------------------------------------------------------------------------
// 11. PROGRAM ENTRY POINT
// -------------------------------------------------------------------------
int main(int argc, char** argv) {
    loadXML("../../Scene/Scene.xml");

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
    glutInitWindowPosition(100, 100);
    glutInitWindowSize(g_windowWidth, g_windowHeight);
    glutCreateWindow("CG@DI-UM – Solar System");

#ifndef __APPLE__
    if (glewInit() != GLEW_OK) {
        cerr << "[FATAL] GLEW initialisation failed" << '\n';
        return EXIT_FAILURE;
    }
#endif

    // load all model files now that the GL context exists ----------------
    for (size_t i = 0; i < g_modelFiles.size(); ++i) {
        char pathm[256] = { 0 };
        char patht[256] = { 0 };
        snprintf(pathm, sizeof(pathm), "../../modelos/%s", g_modelFiles[i].c_str());
        snprintf(patht, sizeof(patht), "../../textures/Teste.jpg");
        loadModelVertices(static_cast<int>(i), pathm, patht);
    }

    // register GLUT callbacks -------------------------------------------
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutSpecialFunc(special);
    glutIdleFunc(display);

    // basic GL state -----------------------------------------------------
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glEnable(GL_TEXTURE_2D);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_NORMALIZE); // caso você use glScalef ou objetos não uniformes

    //glPolygonMode(GL_FRONT, GL_LINE); // wireframe for debug – switch to GL_FILL for final renders
    glClearColor(0.f, 0.f, 0.f, 0.f);

    glutMainLoop();
    return EXIT_SUCCESS;
}
