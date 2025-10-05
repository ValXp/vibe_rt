#include <SFML/Graphics.hpp>
#include <iostream>
#include <cmath>
#include <unistd.h>
#include <fstream>
#include <vector>
#include <pthread.h>
#include <cstdint>

class Vector {
    public:
    Vector(): x(1), y(1), z(1) {}
    Vector(float x, float y, float z): x(x), y(y), z(z) {
    }
    Vector(const Vector& v): x(v.x), y(v.y), z(v.z) {
    }
    // In-place normalize. Nothing if vector is empty.
    Vector normalize() const {
        return *this / (this->length() || 1);
    }
    Vector& operator=(const Vector& v) {
        x = v.x;
        y = v.y;
        z = v.z;
        return *this;
    }
    Vector operator+(const Vector& v) const {
        return Vector(x+v.x, y+v.y, z+v.z);
    }
    Vector operator-(const Vector& v) const {
        return Vector(x-v.x, y-v.y, z-v.z);
    }
    Vector operator-() const {
        return Vector(-x, -y, -z);
    }
    Vector operator*(float f) const {
        return Vector(x * f, y * f, z * f);
    }
    Vector operator/(float f) const {
        return Vector(x / f, y / f, z / f);
    }
    float dot(const Vector& other) const {
        return x * other.x + y * other.y + z * other.z;
    }
    Vector cross(const Vector& other) const {
        return Vector(y * other.z - z * other.y,
                      z * other.x - x * other.z,
                      x * other.y - y * other.x);
    }
    float length() const {
        return sqrt(x*x + y*y + z*z);
    }
    float x, y, z;
};

std::ostream &operator<<(std::ostream &os, const Vector &v) {
    return os << v.x << ", " << v.y << ", " << v.z;
}


class Eye {
    public:
    Eye(float x, float y, float z): position(x, y, z) {
    }
    Eye(const Vector &p): position(p) {
    }
    Vector position;
};

class Screen {
    public:
    Screen(float width, float height, int resolution_x, int resolution_y, const Vector &position):
        width(width), height(height), pixel_size_x(width / resolution_x), pixel_size_y(height / resolution_y), position(position), resolution_x(resolution_x), resolution_y(resolution_y) {
    }
    float width;
    float height;
    float pixel_size_x;
    float pixel_size_y;
    float resolution_x;
    float resolution_y;
    Vector position;
};

class Ray {
    public:
    Ray(const Vector& position, const Vector& direction): position(position), direction((direction - position).normalize()) {
    }
    Vector position;
    Vector direction;
};

class Intersection {
    public:
    Intersection(): worldSpace(), texCoord(), index(0) {}
    Intersection(const Vector& worldSpace, const Vector& texCoord): worldSpace(worldSpace), texCoord(texCoord) {
    }
    Vector worldSpace;
    Vector texCoord;
    int index;
};


bool intersect_triangle_3(const Vector& v0, const Vector& v1, const Vector& v2, const Ray& ray, Intersection &intersection) {
    const float kEpsilon= 0.0000001;
    Vector orig = ray.position;
    Vector dir = ray.direction;
    Vector v0v1 = v1 - v0;
    Vector v0v2 = v2 - v0;
    Vector pvec = dir.cross(v0v2);
    float det = v0v1.dot(pvec);
    // ray and triangle are parallel if det is close to 0
    if (fabs(det) < kEpsilon) return false;
    float invDet = 1 / det;
    Vector tvec = orig - v0;
    float u = tvec.dot(pvec) * invDet;
    if (u < 0.f || u > 1.f) return false;
    Vector qvec = tvec.cross(v0v1);
    float v = dir.dot(qvec) * invDet;
    if (v < 0.f || u + v > 1.f) return false;
    float t = v0v2.dot(qvec) * invDet;
    intersection.texCoord.x = t;
    intersection.texCoord.y = u;
    intersection.texCoord.z = v;
    return true;
}

bool intersect_triangle_2(const Vector& vertex0, const Vector& vertex1, const Vector& vertex2, const Ray& ray) {
    Vector rayOrigin = ray.position;
    Vector rayVector = ray.direction;
    const float EPSILON = 0.0000001;
    Vector edge1 = vertex1 - vertex0;
    Vector edge2 = vertex2 - vertex0;
    Vector h = rayVector.cross(edge2);
    float a = edge1.dot(h);
    if (a > -EPSILON && a < EPSILON)
        return false;    // This ray is parallel to this triangle.
    float f = 1.0/a;
    Vector s = rayOrigin - vertex0;
    float u = f * s.dot(h);
    if (u < 0.0 || u > 1.0)
        return false;
    Vector q = s.cross(edge1);
    float v = f * rayVector.dot(q);
    if (v < 0.0 || u + v > 1.0)
        return false;
    // At this stage we can compute t to find out where the intersection point is on the line.
    float t = f * edge2.dot(q);
    if (t > EPSILON) // ray intersection
    {
        Vector outIntersectionPoint = rayOrigin + rayVector * t;
        return true;
    }
    else // This means that there is a line intersection but not a ray intersection.
        return false;
}

bool intersect_triangle(const Vector& a, const Vector& b, const Vector& c, const Ray& r) {
    Vector n = (b-a).cross(c-a);
    float denom = n.dot(-r.direction);

    if (denom == 0) {
        return false;
    }

    float t = n.dot(r.position - a) / denom;

    float tMin = -0.001;
    float tMax = 0.001;
    if (t < tMin || t > tMax) {
        return false;
    }

    float beta = (r.position - a).cross(c - a).dot(-r.direction) / denom;
    float gamma = (b - a).cross(r.position - a).dot(-r.direction) / denom;

    if (beta < 0 || gamma < 0 ||
      	beta > 1 || gamma > 1
  	    || (beta + gamma) > 1) {
        return false;
    }
/*
    Vector alpha = 1 - beta - gamma;

    // interpolated normal
    Vector ni = alpha * n1 + beta * n2 + gamma * n3;

    // hit point
    Vector p = r.o + (r.d * t);

    result.t = t;
    result.P = p;
    result.N = ni;
    result.material = this->m_material;
*/
    return true;
}


class Sphere {
    public:
    Sphere(const Vector& position, float radius): position(position), radius(radius) {}
    Vector position;
    float radius;
    bool intersect(const Ray& ray) const {
        Vector eye_to_center = position - ray.position;
        Vector eye_ray = (ray.direction + ray.position).normalize();
        float eye_to_intersect_dist = eye_to_center.dot(eye_ray);
        Vector center_proj = (eye_ray * eye_to_intersect_dist) - ray.position;
        //std::cout << " eye_to_c: " << eye_to_center << std::endl;
        //std::cout << " eye_ray: " << eye_ray << std::endl;
        //std::cout << " eye_to_ray_intersect: " << eye_to_intersect_dist << std::endl;
        //std::cout << " c_proj: " << center_proj << std::endl;
        if (eye_to_center.dot(eye_ray) < 0) {
            //std::cout << "Eye on other side!" << std::endl;
            //std::cout << ' ';
        } else {
            float center_to_proj_dist = (position - center_proj).length();
            //std::cout << center_to_proj_dist << std::endl;
            if (center_to_proj_dist > radius) {
                return false;
                std::cout << ' ';
            } else {
                return true;
                std::cout << "X";
            }
        }
        return false;
    }
};

class Model {
    public:
    Model() {}
    Model(std::vector<float> vertices, std::vector<int> indices): vertices(vertices), indices(indices) {
    }

    void verticesAt(int i, Vector& v0, Vector& v1, Vector& v2) {
        int f_index0 = indices[i] - 1;
        int f_index1 = indices[i + 1] - 1;
        int f_index2 = indices[i + 2] - 1;
        v0 = Vector(vertices[f_index0 * 3] + translation.x,
                  vertices[f_index0 * 3 + 1] + translation.y,
                  vertices[f_index0 * 3 + 2] + translation.z);
        v1 = Vector(vertices[f_index1 * 3] + translation.x,
                  vertices[f_index1 * 3 + 1] + translation.y,
                  vertices[f_index1 * 3 + 2] + translation.z);
        v2 = Vector(vertices[f_index2 * 3] + translation.x,
                  vertices[f_index2 * 3 + 1] + translation.y,
                  vertices[f_index2 * 3 + 2] + translation.z);
    }

    bool intersect(const Ray &ray, Intersection &result) {
        for (int i = 0; i < indices.size(); i += 3) {
            Vector v0, v1, v2;
            verticesAt(i, v0, v1, v2);
            if (intersect_triangle_3(v0, v1, v2, ray, result)) {
                result.index = i;
                return true;
            }
        }
        return false;
    }

    float shade(const Intersection& intersection, const Vector& light) {
        // Compute normals on the fly because fuck it. This should be precomputed too.
        Vector v0, v1, v2;
        verticesAt(intersection.index, v0, v1, v2);
        Vector a = v1 - v0;
        Vector b = v2 - v0;
        Vector normal = a.cross(b).normalize();
        Vector lightDir = light - v0; // Grabbing one edge because fuck it again.
        float intensity = normal.dot(lightDir) * 5;
        if (intensity > 1) intensity = 1;
        if (intensity < 0) intensity = 0;
        return intensity;//[int(intensity * char_map_len)];
    }

    Vector translation = Vector(0, -1, 3);
    std::vector<float> vertices;
    std::vector<int> indices;
};

class OBJLoader {
    public:
    static Model parse(const std::string& path) {
        std::ifstream file(path);
        char buff[512];
        std::vector<float> vertices;
        std::vector<int> indices;

        while (file.getline(buff, 512)) {
            auto words = to_words(buff, " \t");
            if (words[0] == "v") {
                float x = std::atof(words[1].data());
                float y = std::atof(words[2].data());
                float z = std::atof(words[3].data());
                vertices.push_back(x);
                vertices.push_back(y);
                vertices.push_back(z);
            } else if (words[0] == "f") {
                //std::cout << words[1] << " " << words[2] << " " << words[3] << std::endl;
                auto a = std::atoi(words[1].data());
                auto b = std::atoi(words[2].data());
                auto c = std::atoi(words[3].data());
                //std::cout << a << " " << b << " " << c << std::endl;
                indices.push_back(a);
                indices.push_back(b);
                indices.push_back(c);
            }
        }
        std::cout << "Loaded " << vertices.size() << " vertices and " << indices.size() << " indices." << std::endl;
        return Model(vertices, indices);
    }


    static std::vector<std::string_view> to_words(std::string_view sv, const std::string& delim) {
        std::vector<std::string_view> words;
        int begin = 0;
        int end = 0;
        while (end < sv.length()) {
            begin = end;
            sv = sv.substr(begin);
            begin = sv.find_first_not_of(delim);
            if (begin < 0) {
                break;
            }
            sv = sv.substr(begin);
            end = sv.find_first_of(delim);
            auto item = sv.substr(0, end);
            words.push_back(item);
        }
        return words;
    }
};

void print_vec(const std::vector<std::string_view>& vec) {
    for (const std::string_view v : vec) {
        std::cout << v << std::endl;
    }
}


template <class W, class R>
class Thread {
    public:
    Thread() : work_container(this) {
    }

    // It's the implementer's responsibility to delete "work" when done.
    R* DoWork(W* work) = 0;

    // We take ownership of "work".
    void Run(W* work) {
        work_container.work = work;
        pthread_create(&_thread, NULL, &Thread::work_fn, dynamic_cast<void*>(&work_container));
    }

    void Join() {
        pthread_join(_thread, NULL);
    }

    private:
    class WorkContainer {
        public:
        WorkContainer(Thread* thread) : thread(thread) {}
        W* work;
        Thread* thread;
    };
    WorkContainer work_container;
    pthread_t _thread;
    static void* work_fn(void *ptr) {
        WorkContainer* work_container = dynamic_cast<WorkContainer*>(ptr);
        void* result = dynamic_cast<void *>(work_container->DoWork(work_container->work));
        // Possible memory leak here. Make sure work is deleted in the 'DoWork' implementation.
        work_container->work = NULL;
        return result;
    }
};

Screen* screen;
Model model;
void init(unsigned int width, unsigned int height, std::string objPath) {
    screen = new Screen(4, 4, width, height, Vector(-4, -3, 20));
    model = OBJLoader::parse(objPath);
}


// Teapot no optim.
// real	0m10.622s
// user	0m10.559s
// sys	0m0.055s
void renderToTexture(std::uint8_t* pixels, float x, float y, float z) {
    static float angle = 0;
    angle += .1f;
    Eye eye(Vector(0, 0, 0));

    screen->position = Vector(x, y, z);
    Vector direction(screen->position);
    Intersection intersection;
    Vector light(cosf(angle) * 5, 0,  sinf(angle) * 5);
    for (int y = 0; y < screen->resolution_y; y++) {
        for (int x = 0; x < screen->resolution_x; x++) {
            direction.x = screen->position.x + x * screen->pixel_size_x;
            direction.y = screen->position.y + y * screen->pixel_size_y;
            Ray ray(eye.position, direction);
            bool intersected = model.intersect(ray, intersection);
            unsigned int r = 50u;
            unsigned int g = 50u;
            unsigned int b = 70u;
            unsigned int a = 255u;

            if (intersected) {
                float shade = model.shade(intersection, light);
                r = 15u+(unsigned int)(150u * shade);
                g = 5u+(unsigned int)(50u * shade);
                b = 25u+(unsigned int)(200u * shade);
                unsigned int a = 255u;
            }
            pixels[y * ((int)screen->resolution_x*4) + x * 4] = r;
            pixels[y * ((int)screen->resolution_x*4) + x * 4 + 1] = g;
            pixels[y * ((int)screen->resolution_x*4) + x * 4 + 2] = b;
            pixels[y * ((int)screen->resolution_x*4) + x * 4 + 3] = a;
        }
    }
}
