#include <SFML/Graphics.hpp>
#include <iostream>
#include <cstdint>
#include <cmath>
#include <filesystem>
#include <optional>
#include <string>

#include "app/CameraController.h"
#include "headers/renderer.h"
#include "headers/thread_pool.hpp"

const uint height = 1000;
const uint width = 1000;

namespace {

void print_usage(const char* prog) {
    std::cout << "Usage: " << prog << " [path/to/model.obj]\n";
}

std::string resolve_obj_path(int argc, char** argv) {
    if (argc > 2) return "";
    if (argc == 2) return argv[1];
    return "resources/bunny_smooth.obj";
}

std::optional<app::Key> mapKey(sf::Keyboard::Key key) {
    switch (key) {
        case sf::Keyboard::Key::W: return app::Key::W;
        case sf::Keyboard::Key::A: return app::Key::A;
        case sf::Keyboard::Key::S: return app::Key::S;
        case sf::Keyboard::Key::D: return app::Key::D;
        case sf::Keyboard::Key::Q: return app::Key::Q;
        case sf::Keyboard::Key::E: return app::Key::E;
        case sf::Keyboard::Key::Left: return app::Key::Left;
        case sf::Keyboard::Key::Right: return app::Key::Right;
        case sf::Keyboard::Key::Up: return app::Key::Up;
        case sf::Keyboard::Key::Down: return app::Key::Down;
        default: return std::nullopt;
    }
}

std::optional<app::MouseButton> mapButton(sf::Mouse::Button button) {
    switch (button) {
        case sf::Mouse::Button::Left: return app::MouseButton::Left;
        case sf::Mouse::Button::Right: return app::MouseButton::Right;
        default: return std::nullopt;
    }
}

}  // namespace

int main(int argc, char** argv)
{
#ifdef __ARM_NEON__
    std::cout << "__ARM_NEON__" << std::endl;
#endif
#ifdef TARGET_FEATURE_NEON
    std::cout << "TARGET_FEATURE_NEON" << std::endl;
#endif
    const char* prog = (argc > 0 && argv && argv[0]) ? argv[0] : "new_rt";
    const std::string objPath = resolve_obj_path(argc, argv);
    if (objPath.empty()) {
        print_usage(prog);
        return EXIT_FAILURE;
    }
    if (!std::filesystem::exists(objPath) || !std::filesystem::is_regular_file(objPath)) {
        std::cerr << "OBJ not found: " << objPath << "\n";
        print_usage(prog);
        return EXIT_FAILURE;
    }
    std::cout << "Loading OBJ: " << objPath << "\n";
    sf::RenderWindow window(sf::VideoMode{sf::Vector2u{width, height}}, "CMake SFML Project");
    window.setFramerateLimit(144);
    Renderer renderer;
    ThreadPool pool; // defaults to hardware_concurrency
    std::cout << "ThreadPool threads: " << pool.thread_count() << "\n";
    renderer.init(width, height, objPath);
    auto* pixels = new std::uint8_t[width * height * 4];
    sf::Texture screenText(sf::Vector2u{width, height});
    screenText.setRepeated(false);
    sf::Sprite sprite{screenText};
    sprite.setTextureRect(sf::IntRect(sf::Vector2i{0, static_cast<int>(height)},
                                      sf::Vector2i{static_cast<int>(width), -static_cast<int>(height)}));
    app::CameraController controller(0.0f, 0.0f, 2.5f);
    controller.setCameraPosition(0.0f, 0.0f, 6.0f);
    controller.setCameraRotation(3.14159265f, 0.0f);
    const int tileW = 64;
    const int tileH = 64;
    while (window.isOpen())
    {
        while (const std::optional<sf::Event> event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>()) {
                window.close();
            }
            if (const auto* key = event->getIf<sf::Event::KeyReleased>()) {
                if (key->code == sf::Keyboard::Key::Escape) {
                    exit(EXIT_SUCCESS);
                }
                if (auto mapped = mapKey(key->code)) {
                    controller.handleKeyReleased(*mapped);
                }
            }
            if (const auto* keyp = event->getIf<sf::Event::KeyPressed>()) {
                if (auto mapped = mapKey(keyp->code)) {
                    controller.handleKeyPressed(*mapped);
                }
            }
            if (const auto* mb = event->getIf<sf::Event::MouseButtonPressed>()) {
                if (auto mapped = mapButton(mb->button)) {
                    controller.handleMousePressed(*mapped, mb->position.x, mb->position.y);
                }
            }
            if (event->is<sf::Event::MouseButtonReleased>()) {
                const auto* mr = event->getIf<sf::Event::MouseButtonReleased>();
                if (mr) {
                    if (auto mapped = mapButton(mr->button)) {
                        controller.handleMouseReleased(*mapped);
                    }
                }
            }
            if (const auto* mm = event->getIf<sf::Event::MouseMoved>()) {
                const bool anyDown = sf::Mouse::isButtonPressed(sf::Mouse::Button::Right)
                                   || sf::Mouse::isButtonPressed(sf::Mouse::Button::Left);
                controller.handleMouseMoved(mm->position.x, mm->position.y, anyDown);
            }
            if (const auto* wheel = event->getIf<sf::Event::MouseWheelScrolled>()) {
                controller.handleMouseWheelScrolled(static_cast<float>(wheel->delta));
            }
        }
        auto pressed = [](sf::Keyboard::Key k){ return sf::Keyboard::isKeyPressed(k); };
        auto scPressed = [](sf::Keyboard::Scancode s){ return sf::Keyboard::isKeyPressed(s); };
        bool w = pressed(sf::Keyboard::Key::W) || scPressed(sf::Keyboard::Scan::W);
        bool s = pressed(sf::Keyboard::Key::S) || scPressed(sf::Keyboard::Scan::S);
        bool d = pressed(sf::Keyboard::Key::D) || scPressed(sf::Keyboard::Scan::D);
        bool a = pressed(sf::Keyboard::Key::A) || scPressed(sf::Keyboard::Scan::A);
        bool e = pressed(sf::Keyboard::Key::E) || scPressed(sf::Keyboard::Scan::E);
        bool q = pressed(sf::Keyboard::Key::Q) || scPressed(sf::Keyboard::Scan::Q);
        controller.setPolledKeys(w, a, s, d, q, e);
        controller.update();
        app::CameraState state = controller.state();
        renderer.setCameraPosition(state.camX, state.camY, state.camZ);
        renderer.setCameraRotation(state.yaw, state.pitch);
        window.clear();
        if (controller.consumeRenderRequest()) {
            renderer.startTiledRender(pixels, state.planeX, state.planeY, state.planeZ, pool, tileW, tileH);
        }
        if (renderer.isRenderFinished()) {
            controller.requestRender();
        }
        screenText.update(pixels);
        window.draw(sprite);
        window.display();
    }
}
