#include <SFML/Graphics.hpp>
#include <iostream>
#include <cstdint>
#include <cmath>
#include <optional>

#include "app/CameraController.h"
#include "headers/renderer.h"
#include "headers/thread_pool.hpp"

const uint height = 1000;
const uint width = 1000;

namespace {

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

int main()
{
#ifdef __ARM_NEON__
    std::cout << "__ARM_NEON__" << std::endl;
#endif
#ifdef TARGET_FEATURE_NEON
    std::cout << "TARGET_FEATURE_NEON" << std::endl;
#endif
    sf::RenderWindow window(sf::VideoMode{sf::Vector2u{width, height}}, "CMake SFML Project");
    window.setFramerateLimit(144);
    Renderer renderer;
    ThreadPool pool; // defaults to hardware_concurrency
    renderer.init(width, height, "resources/bunny_smooth.obj");
    auto* pixels = new std::uint8_t[width * height * 4];
    sf::Texture screenText(sf::Vector2u{width, height});
    screenText.setRepeated(false);
    sf::Sprite sprite{screenText};
    sprite.setTextureRect(sf::IntRect(sf::Vector2i{0, static_cast<int>(height)},
                                      sf::Vector2i{static_cast<int>(width), -static_cast<int>(height)}));
    app::CameraController controller(-1.9f, -2.2f, 2.7f);
    controller.setCameraPosition(0.0f, 0.0f, 0.0f);
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
