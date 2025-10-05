#ifndef OBJ_LOADER_H_
#define OBJ_LOADER_H_

#include <string>
#include <vector>

struct ObjData {
    std::vector<float> vertices;
    std::vector<int> indices;
    std::vector<float> normals;        // packed nx,ny,nz
    std::vector<int> normalIndices;    // 1-based like OBJ, 3 per triangle vertex
};

class ObjLoader {
public:
    static ObjData parse(const std::string& path);
};

#endif // OBJ_LOADER_H_
