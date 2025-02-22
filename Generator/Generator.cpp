#include <iostream>
#include <fstream>
#include <vector>
#include <tuple>
#include <cmath>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Writes a list of (x, y, z) vertices to a file.
void writeToFile(const std::string& filename,
                 const std::vector<std::tuple<float, float, float>>& vertices) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open output file " << filename << std::endl;
        return;
    }

    // (Optional) Write the number of vertices as the first line
    file << vertices.size() << std::endl;
    
    for (const auto& v : vertices) {
        file << std::get<0>(v) << " "
             << std::get<1>(v) << " "
             << std::get<2>(v) << "\n";
    }

    file.close();
}

// Generates a plane on the XZ plane, centered at the origin.
// size: total length of one side of the square
// divisions: number of subdivisions along each axis (the plane will be split into divisions x divisions quads).
void generatePlane(const std::string& filename, float size, int divisions) {
    std::vector<std::tuple<float, float, float>> triangles;
    float half = size / 2.0f;
    float step = size / (float)divisions;

    // Generate two triangles for each quad subdivision
    for (int i = 0; i < divisions; ++i) {
        for (int j = 0; j < divisions; ++j) {
            // Coordinates for corners of the current quad
            float x1 = -half + j * step;
            float z1 = half - i * step;
            float x2 = x1 + step;
            float z2 = z1 - step;

            // The square (quad) corners:
            //  (x1,z1)  ----  (x2,z1)
            //     |              |
            //  (x1,z2)  ----  (x2,z2)

            // Triangle 1: top-left, bottom-left, top-right
            triangles.push_back(std::make_tuple(x1, 0.0f, z1));
            triangles.push_back(std::make_tuple(x1, 0.0f, z2));
            triangles.push_back(std::make_tuple(x2, 0.0f, z1));

            // Triangle 2: bottom-left, bottom-right, top-right
            triangles.push_back(std::make_tuple(x1, 0.0f, z2));
            triangles.push_back(std::make_tuple(x2, 0.0f, z2));
            triangles.push_back(std::make_tuple(x2, 0.0f, z1));
        }
    }

    writeToFile(filename, triangles);
}

// Generates a box centered at the origin.
// size: the length of an edge of the cube
// divisions: number of subdivisions along each edge of the cube
void generateBox(const std::string& filename, float size, int divisions) {
    std::vector<std::tuple<float, float, float>> vertices;
    
    // If you only want one face per side without subdivisions, ignore `divisions`.
    // If you want to implement subdivisions, we do it similarly to the plane,
    // but in 3D. For Phase 1, you can simplify to just 1 face (divisions=1).
    
    float half = size / 2.0f;
    float step = size / (float)divisions;

    // We will generate each face in (divisions x divisions) quads, each producing 2 triangles.
    // The faces: front, back, left, right, top, bottom.

    // Lambda to help push back two triangles forming one quad on a given plane.
    auto pushQuad = [&](float x1, float y1, float z1,
                        float x2, float y2, float z2,
                        float x3, float y3, float z3,
                        float x4, float y4, float z4) {
        // Triangle 1
        vertices.emplace_back(x1, y1, z1);
        vertices.emplace_back(x2, y2, z2);
        vertices.emplace_back(x3, y3, z3);

        // Triangle 2
        vertices.emplace_back(x3, y3, z3);
        vertices.emplace_back(x2, y2, z2);
        vertices.emplace_back(x4, y4, z4);
    };

    // For each face, we step through a grid of size divisions x divisions.
    // FRONT FACE (z = +half), going from -half to +half in x and y
    for(int i = 0; i < divisions; ++i){
        for(int j = 0; j < divisions; ++j){
            float x1 = -half + j * step;
            float y1 = -half + i * step;
            float x2 = x1 + step;
            float y2 = y1 + step;
            float z = half; // front face

            pushQuad( x1, y1, z,
                      x2, y1, z,
                      x1, y2, z,
                      x2, y2, z );
        }
    }

    // BACK FACE (z = -half)
    for(int i = 0; i < divisions; ++i){
        for(int j = 0; j < divisions; ++j){
            float x1 = -half + j * step;
            float y1 = -half + i * step;
            float x2 = x1 + step;
            float y2 = y1 + step;
            float z = -half; // back face

            // Note the order to keep consistent winding (or invert if needed).
            pushQuad( x2, y1, z,
                      x1, y1, z,
                      x2, y2, z,
                      x1, y2, z );
        }
    }

    // LEFT FACE (x = -half)
    for(int i = 0; i < divisions; ++i){
        for(int j = 0; j < divisions; ++j){
            float z1 = half - j * step;
            float y1 = -half + i * step;
            float z2 = z1 - step;
            float y2 = y1 + step;
            float x = -half;

            pushQuad( x, y1, z1,
                      x, y1, z2,
                      x, y2, z1,
                      x, y2, z2 );
        }
    }

    // RIGHT FACE (x = +half)
    for(int i = 0; i < divisions; ++i){
        for(int j = 0; j < divisions; ++j){
            float z1 = half - j * step;
            float y1 = -half + i * step;
            float z2 = z1 - step;
            float y2 = y1 + step;
            float x = half;

            // Reverse the order to keep consistent winding
            pushQuad( x, y1, z2,
                      x, y1, z1,
                      x, y2, z2,
                      x, y2, z1 );
        }
    }

    // TOP FACE (y = +half)
    for(int i = 0; i < divisions; ++i){
        for(int j = 0; j < divisions; ++j){
            float x1 = -half + j * step;
            float z1 = half - i * step;
            float x2 = x1 + step;
            float z2 = z1 - step;
            float y = half;

            pushQuad( x1, y, z1,
                      x2, y, z1,
                      x1, y, z2,
                      x2, y, z2 );
        }
    }

    // BOTTOM FACE (y = -half)
    for(int i = 0; i < divisions; ++i){
        for(int j = 0; j < divisions; ++j){
            float x1 = -half + j * step;
            float z1 = half - i * step;
            float x2 = x1 + step;
            float z2 = z1 - step;
            float y = -half;

            // Reverse order for consistent winding
            pushQuad( x2, y, z1,
                      x1, y, z1,
                      x2, y, z2,
                      x1, y, z2 );
        }
    }

    writeToFile(filename, vertices);
}

// Generates a sphere centered at the origin.
// radius: Sphere radius
// slices: number of vertical divisions
// stacks: number of horizontal divisions
void generateSphere(const std::string& filename, float radius, int slices, int stacks) {
    std::vector<std::tuple<float, float, float>> triangles;

    // We first create a grid of (slices+1)*(stacks+1) vertices in spherical coordinates.
    // Then we connect adjacent vertices to form triangles.

    // Store the vertices in a 2D array-like structure for easy indexing
    std::vector<std::vector<std::tuple<float, float, float>>> grid(stacks + 1,
        std::vector<std::tuple<float, float, float>>(slices + 1));

    for (int stack = 0; stack <= stacks; ++stack) {
        // phi goes from 0 (top) to pi (bottom)
        float phi = M_PI * float(stack) / float(stacks);
        for (int slice = 0; slice <= slices; ++slice) {
            // theta goes around the sphere from 0 to 2*pi
            float theta = 2.0f * M_PI * float(slice) / float(slices);
            float x = radius * sinf(phi) * cosf(theta);
            float y = radius * cosf(phi);
            float z = radius * sinf(phi) * sinf(theta);
            grid[stack][slice] = std::make_tuple(x, y, z);
        }
    }

    // Connect these vertices with two triangles per "quad" in the grid.
    for (int stack = 0; stack < stacks; ++stack) {
        for (int slice = 0; slice < slices; ++slice) {
            // Indices for the four corners of the quad
            auto p1 = grid[stack][slice];
            auto p2 = grid[stack + 1][slice];
            auto p3 = grid[stack][slice + 1];
            auto p4 = grid[stack + 1][slice + 1];

            // First triangle
            triangles.push_back(p1);
            triangles.push_back(p2);
            triangles.push_back(p3);

            // Second triangle
            triangles.push_back(p3);
            triangles.push_back(p2);
            triangles.push_back(p4);
        }
    }

    writeToFile(filename, triangles);
}

// Generates a cone with the base centered on the XZ-plane and its apex along +Y.
// radius: radius of the base
// height: cone height
// slices: divisions around the axis
// stacks: divisions from the base to the apex
void generateCone(const std::string& filename, float radius, float height,
                  int slices, int stacks) {
    std::vector<std::tuple<float, float, float>> vertices;

    // 1) Generate the base (a circle on the XZ plane at y = 0).
    //    We do a fan from the center (0,0,0) to each triangle around the circle.

    float theta_step = 2.0f * M_PI / slices;
    for (int i = 0; i < slices; ++i) {
        float theta = i * theta_step;
        float nextTheta = (i + 1) * theta_step;
        // Center
        vertices.push_back(std::make_tuple(0.0f, 0.0f, 0.0f));
        // Current point on circle
        vertices.push_back(std::make_tuple(radius * cosf(theta),
                                           0.0f,
                                           radius * sinf(theta)));
        // Next point on circle
        vertices.push_back(std::make_tuple(radius * cosf(nextTheta),
                                           0.0f,
                                           radius * sinf(nextTheta)));
    }

    // 2) Generate the side surface.
    //    We'll slice the cone in "stacks" along its height, forming ring-like cross-sections
    //    from y=0 (radius = R) up to y=height (radius = 0).
    //    Each ring is an approximation for that height segment.

    // For each stack level, we have a smaller circle, until we reach the apex.
    // radius decreases linearly, from 'radius' at y=0 to 0 at y=height.

    float stackHeight = height / stacks;
    for (int st = 0; st < stacks; ++st) {
        float y_lower = st * stackHeight;            // base of current ring
        float y_upper = (st + 1) * stackHeight;      // top of current ring

        float r_lower = radius * (1.0f - (float)st / stacks);       // radius at y_lower
        float r_upper = radius * (1.0f - (float)(st + 1) / stacks); // radius at y_upper

        for (int s = 0; s < slices; ++s) {
            float theta = s * theta_step;
            float nextTheta = (s + 1) * theta_step;

            // The four corners of the quad on the ring
            std::tuple<float, float, float> p1 = {
                r_lower * cosf(theta), y_lower, r_lower * sinf(theta)
            };
            std::tuple<float, float, float> p2 = {
                r_lower * cosf(nextTheta), y_lower, r_lower * sinf(nextTheta)
            };
            std::tuple<float, float, float> p3 = {
                r_upper * cosf(theta), y_upper, r_upper * sinf(theta)
            };
            std::tuple<float, float, float> p4 = {
                r_upper * cosf(nextTheta), y_upper, r_upper * sinf(nextTheta)
            };

            // Two triangles for each quad
            // Triangle 1: p1, p2, p3
            vertices.push_back(p1);
            vertices.push_back(p2);
            vertices.push_back(p3);

            // Triangle 2: p3, p2, p4
            vertices.push_back(p3);
            vertices.push_back(p2);
            vertices.push_back(p4);
        }
    }

    writeToFile(filename, vertices);
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
                  << "  " << argv[0] << " plane <size> <divisions> <output_file>\n"
                  << "  " << argv[0] << " box <size> <divisions> <output_file>\n"
                  << "  " << argv[0] << " sphere <radius> <slices> <stacks> <output_file>\n"
                  << "  " << argv[0] << " cone <radius> <height> <slices> <stacks> <output_file>\n";
        return 1;
    }

    std::string shape = argv[1];

    try {
        if (shape == "plane") {
            if (argc != 5) {
                std::cerr << "Error: Wrong number of arguments for plane.\n";
                return 1;
            }
            float size = std::stof(argv[2]);
            int divisions = std::stoi(argv[3]);
            std::string outFile = argv[4];
            generatePlane(outFile, size, divisions);
        }
        else if (shape == "box") {
            if (argc != 5) {
                std::cerr << "Error: Wrong number of arguments for box.\n";
                return 1;
            }
            float size = std::stof(argv[2]);
            int divisions = std::stoi(argv[3]);
            std::string outFile = argv[4];
            generateBox(outFile, size, divisions);
        }
        else if (shape == "sphere") {
            if (argc != 6) {
                std::cerr << "Error: Wrong number of arguments for sphere.\n";
                return 1;
            }
            float radius = std::stof(argv[2]);
            int slices = std::stoi(argv[3]);
            int stacks = std::stoi(argv[4]);
            std::string outFile = argv[5];
            generateSphere(outFile, radius, slices, stacks);
        }
        else if (shape == "cone") {
            if (argc != 7) {
                std::cerr << "Error: Wrong number of arguments for cone.\n";
                return 1;
            }
            float radius = std::stof(argv[2]);
            float height = std::stof(argv[3]);
            int slices = std::stoi(argv[4]);
            int stacks = std::stoi(argv[5]);
            std::string outFile = argv[6];
            generateCone(outFile, radius, height, slices, stacks);
        }
        else {
            std::cerr << "Error: Unknown shape '" << shape << "'.\n";
            return 1;
        }
    }
    catch (const std::exception& e) {
        std::cerr << "Error parsing arguments: " << e.what() << std::endl;
        return 1;
    }

    std::cout << "File generated successfully.\n";
    return 0;
}
