#include "headers/obj_loader.h"

#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>

namespace {

std::string write_temp_obj(const std::string& contents) {
    auto now = std::chrono::steady_clock::now().time_since_epoch().count();
    std::filesystem::path path = std::filesystem::temp_directory_path()
        / ("obj_loader_test_" + std::to_string(now) + ".obj");
    std::ofstream out(path);
    out << contents;
    out.close();
    return path.string();
}

}  // namespace

TEST_CASE("ObjLoader parses vertices and normals") {
    std::string path = write_temp_obj(
        "v 1 2 3\n"
        "v 4 5 6\n"
        "vn 0 1 0\n"
        "vn 1 0 0\n"
    );
    ObjData data = ObjLoader::parse(path);
    REQUIRE(data.vertices.size() == 6);
    REQUIRE(data.normals.size() == 6);
    CHECK(data.vertices[0] == 1.0f);
    CHECK(data.vertices[4] == 5.0f);
    CHECK(data.normals[1] == 1.0f);
}

TEST_CASE("ObjLoader parses face formats") {
    std::string path = write_temp_obj(
        "v 0 0 0\n"
        "v 1 0 0\n"
        "v 0 1 0\n"
        "vn 0 0 1\n"
        "vn 0 1 0\n"
        "vn 1 0 0\n"
        "f 1 2 3\n"
        "f 1//1 2//2 3//3\n"
        "f 1/4 2/5 3/6\n"
        "f 1/4/1 2/5/2 3/6/3\n"
    );
    ObjData data = ObjLoader::parse(path);
    REQUIRE(data.indices.size() == 12);
    REQUIRE(data.normalIndices.size() == 12);
    CHECK(data.indices[0] == 1);
    CHECK(data.normalIndices[0] == 0);
    CHECK(data.normalIndices[3] == 1);
    CHECK(data.normalIndices[9] == 1);
}
