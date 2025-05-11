#include <iostream>
#include <fstream>
#include <vector>
#include <tuple>
#include <cmath>
#include <string>
#include <stdexcept>
#include <algorithm>    // for std::replace
#include <sstream>      // for std::istringstream
#include <limits>       // for std::numeric_limits


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
using Vec3 = std::tuple<float, float, float>;


struct Vertex {
    float x, y, z;      // position
    float nx, ny, nz;   // normal
    float u, v;         // UV
};
using VertList = std::vector<Vertex>;

inline void pushTri(VertList& v,
                    const Vertex& a,
                    const Vertex& b,
                    const Vertex& c) {
    v.push_back(a); v.push_back(b); v.push_back(c);
}


void writeToFile(const std::string& filename, const VertList& v) {
    std::ofstream f(filename);
    if (!f) throw std::runtime_error("cannot open " + filename);

    f << v.size() << '\n';
    for (const auto& p : v)
        f << p.x  << ' ' << p.y  << ' ' << p.z  << ' '
          << p.nx << ' ' << p.ny << ' ' << p.nz << ' '
          << p.u  << ' ' << p.v  << '\n';
}


// Generates a plane on the XZ plane, centered at the origin.
// size: total length of one side of the square
// divisions: number of subdivisions along each axis (the plane will be split into divisions x divisions quads).
void generatePlane(const std::string& filename,float size,int divisions){
    const float half=size/2.0f, step=size/divisions;
    VertList v;

    for(int i=0;i<divisions;++i)
        for(int j=0;j<divisions;++j){
            float x0=-half+j*step, x1=x0+step;
            float z0= half-i*step, z1=z0-step;
            // normals & uv
            Vertex a{x0,0,z1, 0,1,0, float(j)/divisions,     float(i+1)/divisions};
            Vertex b{x0,0,z0, 0,1,0, float(j)/divisions,     float(i)/divisions};
            Vertex c{x1,0,z0, 0,1,0, float(j+1)/divisions,   float(i)/divisions};
            Vertex d{x1,0,z1, 0,1,0, float(j+1)/divisions,   float(i+1)/divisions};
            pushTri(v,a,b,c);
            pushTri(v,c,d,a);
        }
    writeToFile(filename,v);
}


// Generates a box centered at the origin.
// size: the length of an edge of the cube
// divisions: number of subdivisions along each edge of the cube
void generateBox(const std::string& filename,float size,int divisions){
    const float h=size/2.0f, step=size/divisions;
    VertList v;

    auto face=[&](float nx,float ny,float nz,      // normal
                  float ux,float uy,float uz,      // u-axis
                  float vx,float vy,float vz,      // v-axis
                  float ox,float oy,float oz){     // origin (lower-left corner)
        for(int i=0;i<divisions;++i)
            for(int j=0;j<divisions;++j){
                float u0=j*step,   v0=i*step;
                float u1=u0+step,  v1=v0+step;
                auto make=[&](float u,float v){
                    float x=ox+ux*u+vx*v;
                    float y=oy+uy*u+vy*v;
                    float z=oz+uz*u+vz*v;
                    return Vertex{x,y,z, nx,ny,nz,
                                   u/size, v/size};
                };
                Vertex a=make(u0,v1), b=make(u0,v0),
                       c=make(u1,v0), d=make(u1,v1);
                pushTri(v,a,b,c);
                pushTri(v,c,d,a);
            }
    };

    // +Z (front)
    face(0,0,1,  1,0,0, 0,1,0, -h,-h, h);
    // -Z (back)
    face(0,0,-1, -1,0,0,0,1,0,  h,-h,-h);
    // -X (left)
    // LEFT  (x = -1, z goes +1 → -1)
    face(-1,0,0,  0,0, 1, 0,1,0,  -h,-h, h);

    // RIGHT (x = +1, z goes -1 → +1)
    face( 1,0,0,  0,0,-1, 0,1,0,   h,-h,-h);

    // +Y (top)
    face(0,1,0,  1,0,0, 0,0,-1,-h, h, h);
    // -Y (bottom)
    face(0,-1,0, 1,0,0, 0,0,1, -h,-h,-h);

    writeToFile(filename,v);
}


// Generates a sphere centered at the origin.
// radius: Sphere radius
// slices: number of vertical divisions
// stacks: number of horizontal divisions
void generateSphere(const std::string& filename,float r,int slices,int stacks){
    std::vector<std::vector<Vertex>> grid(stacks+1,std::vector<Vertex>(slices+1));
    for(int i=0;i<=stacks;++i){
        float phi=M_PI*i/stacks;
        for(int j=0;j<=slices;++j){
            float theta=2*M_PI*j/slices;
            float x=r*sinf(phi)*cosf(theta),
                  y=r*cosf(phi),
                  z=r*sinf(phi)*sinf(theta);
            float nx=x/r, ny=y/r, nz=z/r;
            float u=theta/(2*M_PI), v=phi/M_PI;
            grid[i][j]={x,y,z, nx,ny,nz, u,v};
        }
    }

    VertList v;
    for(int i=0;i<stacks;++i)
        for(int j=0;j<slices;++j){
            Vertex p1=grid[i][j],
                   p2=grid[i+1][j],
                   p3=grid[i][j+1],
                   p4=grid[i+1][j+1];
            pushTri(v,p2,p1,p3);
            pushTri(v,p3,p4,p2);
        }
    writeToFile(filename,v);
}


// Generates a cone with the base centered on the XZ-plane and its apex along +Y.
// radius: radius of the base
// height: cone height
// slices: divisions around the axis
// stacks: divisions from the base to the apex
void generateCone(const std::string& filename,float R,float H,int slices,int stacks){
    VertList v;
    const float dTheta=2*M_PI/slices;

    // ---- base (y=0, normal 0,-1,0) ----
    for(int i=0;i<slices;++i){
        float t0=i*dTheta, t1=(i+1)*dTheta;
        Vertex c{0,0,0, 0,-1,0, 0.5f,0.5f};
        Vertex a{R*cosf(t0),0,R*sinf(t0), 0,-1,0,
                 0.5f+0.5f*cosf(t0), 0.5f+0.5f*sinf(t0)};
        Vertex b{R*cosf(t1),0,R*sinf(t1), 0,-1,0,
                 0.5f+0.5f*cosf(t1), 0.5f+0.5f*sinf(t1)};
        pushTri(v,a,b,c);
    }

    // ---- side ----
    float stackH=H/stacks;
    for(int st=0;st<stacks;++st){
        float y0=st*stackH,     y1=(st+1)*stackH;
        float r0=R*(1.f - st/float(stacks)),
              r1=R*(1.f - (st+1)/float(stacks));
        for(int s=0;s<slices;++s){
            float t0=s*dTheta, t1=(s+1)*dTheta;
            // positions
            auto P=[&](float r,float y,float t){
                return Vec3{r*cosf(t), y, r*sinf(t)};
            };
            // vertices with normals & UV
            auto make=[&](float r,float y,float t){
                float x=r*cosf(t), z=r*sinf(t);
                float nx=x, ny=R/H, nz=z;
                float len=sqrtf(nx*nx+ny*ny+nz*nz);
                nx/=len; ny/=len; nz/=len;
                float u=t/(2*M_PI);   // <<< use t, not t0
                float v=y/H;
                return Vertex{x,y,z, nx,ny,nz, u,v};
            };
            
            Vertex p1=make(r0,y0,t0);
            Vertex p2=make(r0,y0,t1);
            Vertex p3=make(r1,y1,t0);
            Vertex p4=make(r1,y1,t1);
            pushTri(v,p2,p1,p3);
            pushTri(v,p3,p4,p2);
        }
    }
    writeToFile(filename,v);
}


// Read all control points from a file: groups of 16 (4x4) points per patch
std::vector<std::vector<Vec3>> loadBezierPatches(const std::string& ctrlFile) {
    std::ifstream in(ctrlFile);
    if (!in.is_open())
        throw std::runtime_error("Cannot open control file " + ctrlFile);

    int numPatches;
    in >> numPatches;
    in.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // skip to end‑of‑line

    // Read patch definitions (each line: 16 integers separated by commas)
    std::vector<std::vector<int>> patchIdx(numPatches, std::vector<int>(16));
    for (int p = 0; p < numPatches; ++p) {
        std::string line;
        std::getline(in, line);
        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream ss(line);
        for (int i = 0; i < 16; ++i)
            ss >> patchIdx[p][i];
    }

    // Read control‑point count
    int numPoints;
    in >> numPoints;
    in.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    // Read that many x,y,z triples (comma separated)
    std::vector<Vec3> points(numPoints);
    for (int i = 0; i < numPoints; ++i) {
        std::string line;
        std::getline(in, line);
        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream ss(line);
        float x,y,z;
        ss >> x >> y >> z;
        points[i] = std::make_tuple(x,y,z);
    }

    // Build the patches of Vec3
    std::vector<std::vector<Vec3>> result(numPatches, std::vector<Vec3>(16));
    for (int p = 0; p < numPatches; ++p) {
        for (int i = 0; i < 16; ++i) {
            int idx = patchIdx[p][i];
            if (idx < 0 || idx >= numPoints)
                throw std::runtime_error("Patch index out of range");
            result[p][i] = points[idx];
        }
    }

    return result;
}

// Evaluate the i-th Bernstein polynomial (degree 3) at t
inline float bernstein(int i, float t) {
    float u = 1.0f - t;
    switch (i) {
        case 0: return u * u * u;
        case 1: return 3 * t * u * u;
        case 2: return 3 * t * t * u;
        case 3: return t * t * t;
    }
    return 0.0f;
}

 //Evaluate a single patch at parameters (u,v)
Vec3 evalPatch(const std::vector<Vec3>& cp, float u, float v) {
    float bu[4], bv[4];
    for (int i = 0; i < 4; ++i) {
        bu[i] = bernstein(i, u);
        bv[i] = bernstein(i, v);
    }
    float x = 0, y = 0, z = 0;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            float b = bu[i] * bv[j];
            auto& p = cp[i*4 + j];
            x += b * std::get<0>(p);
            y += b * std::get<1>(p);
            z += b * std::get<2>(p);
        }
    }
    return std::make_tuple(x, y, z);
}

void generateBezier(const std::string& filename,
    const std::string& ctrlFile,
    int tessellation) {
    auto patches = loadBezierPatches(ctrlFile);
    VertList tris;          // was std::vector<Vec3>


    // For each patch:
    for (const auto& cp : patches) {
        for (int iu = 0; iu < tessellation; ++iu) {
            for (int iv = 0; iv < tessellation; ++iv) {
            float u0 = iu / float(tessellation);
            float v0 = iv / float(tessellation);
            float u1 = (iu + 1) / float(tessellation);
            float v1 = (iv + 1) / float(tessellation);

            Vec3 p00 = evalPatch(cp, u0, v0);
            Vec3 p10 = evalPatch(cp, u1, v0);
            Vec3 p11 = evalPatch(cp, u1, v1);
            Vec3 p01 = evalPatch(cp, u0, v1);

            auto makeV=[&](const Vec3& P,float u,float v){
                float x,y,z; std::tie(x,y,z)=P;
                // normal via finite diff
                const float eps=1e-3f;
                Vec3 du=evalPatch(cp,u+eps,v), dv=evalPatch(cp,u,v+eps);
                float ux,uy,uz,vx,vy,vz;
                std::tie(ux,uy,uz)=du; std::tie(vx,vy,vz)=dv;
                float nx=uy*uz - uz*vy,
                      ny=uz*vx - ux*uz,
                      nz=ux*vy - uy*vx;
                float len=sqrtf(nx*nx+ny*ny+nz*nz);
                if(len>0){ nx/=len; ny/=len; nz/=len; }
                return Vertex{x,y,z, nx,ny,nz, u,v};
            };
            Vertex v00=makeV(p00,u0,v0), v10=makeV(p10,u1,v0);
            Vertex v11=makeV(p11,u1,v1), v01=makeV(p01,u0,v1);
            pushTri(tris,v00,v10,v11);
            pushTri(tris,v00,v11,v01);
            
            }
        }
    }

    writeToFile(filename,tris);

}


// MAIN FUNCTION
// Usage examples (Phase 1):
//   generator plane <size> <divisions> <output_file>
//   generator box <size> <divisions> <output_file>
//   generator sphere <radius> <slices> <stacks> <output_file>
//   generator cone <radius> <height> <slices> <stacks> <output_file>
int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cout << "Usage:\n"
                  << "  " << argv[0] << " plane <size> <divisions> <output>\n"
                  << "  " << argv[0] << " box <size> <divisions> <output>\n"
                  << "  " << argv[0] << " sphere <radius> <slices> <stacks> <output>\n"
                  << "  " << argv[0] << " cone <radius> <height> <slices> <stacks> <output>\n"
                  << "  " << argv[0] << " bezier <ctrl_file> <tess> <output>\n";
        return 1;
    }

    std::string shape = argv[1];
    try {
        if (shape == "plane") {
            float size = std::stof(argv[2]);
            int divs  = std::stoi(argv[3]);
            generatePlane(argv[4], size, divs);
        }
        else if (shape == "box") {
            float size = std::stof(argv[2]);
            int divs  = std::stoi(argv[3]);
            generateBox(argv[4], size, divs);
        }
        else if (shape == "sphere") {
            float r = std::stof(argv[2]);
            int sl = std::stoi(argv[3]);
            int st = std::stoi(argv[4]);
            generateSphere(argv[5], r, sl, st);
        }
        else if (shape == "cone") {
            float r = std::stof(argv[2]);
            float h = std::stof(argv[3]);
            int sl = std::stoi(argv[4]);
            int st = std::stoi(argv[5]);
            generateCone(argv[6], r, h, sl, st);
        }
        else if (shape == "bezier") {
            std::string ctrlFile = argv[2];
            int tess = std::stoi(argv[3]);
            std::string outFile = argv[4];
            generateBezier(outFile, ctrlFile, tess);
        }
        else {
            throw std::runtime_error("Unknown shape: " + shape);
        }
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    std::cout << "File generated successfully." << std::endl;
    return 0;
}

