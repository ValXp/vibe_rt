#include "headers/obj_loader.h"

#include <fstream>
#include <iostream>
#include <string_view>

static std::vector<std::string_view> to_words(std::string_view sv, const std::string& delim) {
    std::vector<std::string_view> words;
    size_t begin = 0;
    size_t end = 0;
    while (end < sv.length()) {
        begin = end;
        sv = sv.substr(begin);
        begin = sv.find_first_not_of(delim);
        if (begin == std::string_view::npos) {
            break;
        }
        sv = sv.substr(begin);
        end = sv.find_first_of(delim);
        auto item = sv.substr(0, end);
        words.push_back(item);
        if (end == std::string_view::npos) {
            break;
        }
    }
    return words;
}

static inline void split_slash(const std::string& token, int& v, int& vt, int& vn) {
    // Parse formats: v, v//vn, v/vt, v/vt/vn
    v = vt = vn = 0;
    size_t p1 = token.find('/');
    if (p1 == std::string::npos) {
        v = std::atoi(token.c_str());
        return;
    }
    std::string s0 = token.substr(0, p1);
    v = std::atoi(s0.c_str());
    size_t p2 = token.find('/', p1 + 1);
    if (p2 == std::string::npos) {
        // v/vt
        std::string s1 = token.substr(p1 + 1);
        vt = s1.empty() ? 0 : std::atoi(s1.c_str());
        return;
    }
    // v/vt/vn or v//vn
    std::string s1 = token.substr(p1 + 1, p2 - (p1 + 1));
    std::string s2 = token.substr(p2 + 1);
    vt = s1.empty() ? 0 : std::atoi(s1.c_str());
    vn = s2.empty() ? 0 : std::atoi(s2.c_str());
}

ObjData ObjLoader::parse(const std::string& path) {
    std::ifstream file(path);
    char buff[512];
    ObjData data;

    while (file.getline(buff, 512)) {
        auto words = to_words(buff, " \t");
        if (words.empty()) continue;
        if (words[0] == "v") {
            float x = std::atof(std::string(words[1]).c_str());
            float y = std::atof(std::string(words[2]).c_str());
            float z = std::atof(std::string(words[3]).c_str());
            data.vertices.push_back(x);
            data.vertices.push_back(y);
            data.vertices.push_back(z);
        } else if (words[0] == "vn") {
            float nx = std::atof(std::string(words[1]).c_str());
            float ny = std::atof(std::string(words[2]).c_str());
            float nz = std::atof(std::string(words[3]).c_str());
            data.normals.push_back(nx);
            data.normals.push_back(ny);
            data.normals.push_back(nz);
        } else if (words[0] == "f") {
            // Expect at least 3 vertices per face
            // Parse first 3 tokens as a triangle; ignore ngons
            int v[3] = {0,0,0};
            int vt[3] = {0,0,0};
            int vn[3] = {0,0,0};
            split_slash(std::string(words[1]), v[0], vt[0], vn[0]);
            split_slash(std::string(words[2]), v[1], vt[1], vn[1]);
            split_slash(std::string(words[3]), v[2], vt[2], vn[2]);
            data.indices.push_back(v[0]);
            data.indices.push_back(v[1]);
            data.indices.push_back(v[2]);
            data.normalIndices.push_back(vn[0]);
            data.normalIndices.push_back(vn[1]);
            data.normalIndices.push_back(vn[2]);
        }
    }
    std::cout << "Loaded " << data.vertices.size() << " vertices and " << data.indices.size() << " indices." << std::endl;
    return data;
}
