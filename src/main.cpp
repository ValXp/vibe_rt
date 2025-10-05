#include <SFML/Graphics.hpp>
#include <iostream>
#include <cstdint>
#include <cmath>

#include "headers/renderer.h"
#include "headers/thread_pool.hpp"

const uint height = 1000;
const uint width = 1000;

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
    float x, y, z; // image-plane pan (x,y) and focal distance z
    x = -1.9f;
    y = -2.2f;
    z = 2.7f;
    // camera state
    float camX = 0.f, camY = 0.f, camZ = 0.f;
    // camera rotation state (radians)
    float yaw = 0.0f;
    float pitch = 0.0f;
    bool rotating = false;
    sf::Vector2i lastMouse{0, 0};
    // movement state (tracked via events, also polled as fallback)
    bool kW = false, kA = false, kS = false, kD = false, kQ = false, kE = false;
    bool renderRequested = true; // kick off initial render
    const int tileW = 64;
    const int tileH = 64;
    // initialize camera in renderer
    renderer.setCameraPosition(camX, camY, camZ);
    while (window.isOpen())
    {
        while (const std::optional<sf::Event> event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>()) {
                window.close();
            }
            if (const auto* key = event->getIf<sf::Event::KeyReleased>()) {
                switch (key->code) {
                    case sf::Keyboard::Key::Left:
                        x -= .1f;
                        renderRequested = true;
                        break;
                    case sf::Keyboard::Key::Right:
                        x += .1f;
                        renderRequested = true;
                        break;
                    case sf::Keyboard::Key::Down:
                        y -= .1f;
                        renderRequested = true;
                        break;
                    case sf::Keyboard::Key::Up:
                        y += .1f;
                        renderRequested = true;
                        break;
                    case sf::Keyboard::Key::Escape:
                        exit(EXIT_SUCCESS);
                        break;
                    case sf::Keyboard::Key::W: kW = false; break;
                    case sf::Keyboard::Key::A: kA = false; break;
                    case sf::Keyboard::Key::S: kS = false; break;
                    case sf::Keyboard::Key::D: kD = false; break;
                    case sf::Keyboard::Key::Q: kQ = false; break;
                    case sf::Keyboard::Key::E: kE = false; break;
                    default:
                        break;
                }
                std::cout << "x " << x << " y " << y << std::endl;
            }
            if (const auto* keyp = event->getIf<sf::Event::KeyPressed>()) {
                switch (keyp->code) {
                    case sf::Keyboard::Key::W: kW = true; break;
                    case sf::Keyboard::Key::A: kA = true; break;
                    case sf::Keyboard::Key::S: kS = true; break;
                    case sf::Keyboard::Key::D: kD = true; break;
                    case sf::Keyboard::Key::Q: kQ = true; break;
                    case sf::Keyboard::Key::E: kE = true; break;
                    default: break;
                }
            }
            if (const auto* mb = event->getIf<sf::Event::MouseButtonPressed>()) {
                if (mb->button == sf::Mouse::Button::Right || mb->button == sf::Mouse::Button::Left) {
                    rotating = true;
                    lastMouse = mb->position;
                }
            }
            if (event->is<sf::Event::MouseButtonReleased>()) {
                const auto* mr = event->getIf<sf::Event::MouseButtonReleased>();
                if (mr && (mr->button == sf::Mouse::Button::Right || mr->button == sf::Mouse::Button::Left)) {
                    rotating = false;
                }
            }
            if (const auto* mm = event->getIf<sf::Event::MouseMoved>()) {
                // Trackpad users: also check if a button is currently held
                const bool anyDown = sf::Mouse::isButtonPressed(sf::Mouse::Button::Right)
                                   || sf::Mouse::isButtonPressed(sf::Mouse::Button::Left);
                if (rotating || anyDown) {
                    sf::Vector2i pos = mm->position;
                    int dx = pos.x - lastMouse.x;
                    int dy = pos.y - lastMouse.y;
                    lastMouse = pos;
                    const float sensitivity = 0.0035f; // rad per pixel
                    yaw += dx * sensitivity;
                    pitch -= dy * sensitivity;
                    renderer.setCameraRotation(yaw, pitch);
                    renderRequested = true;
                }
            }
            if (const auto* wheel = event->getIf<sf::Event::MouseWheelScrolled>()) {
                // Two-finger scroll rotates yaw as a fallback (SFML wheel orientation varies by version)
                const float wsens = 0.02f;
                yaw += static_cast<float>(wheel->delta) * wsens;
                renderer.setCameraRotation(yaw, pitch);
                renderRequested = true;
            }
        }
        // Continuous WASD movement (camera space)
        bool moved = false;
        float yawCur, pitchCur;
        renderer.getCameraRotation(yawCur, pitchCur);
        // camera basis from yaw/pitch (match renderer):
        const float cy = std::cos(yawCur), sy = std::sin(yawCur);
        const float cp = std::cos(pitchCur), sp = std::sin(pitchCur);
        // forward
        float fx = sy * cp;
        float fy = sp;
        float fz = cy * cp;
        // right on ground plane (ignore pitch) for stable strafing
        float rx = cy;
        float ry = 0.f;
        float rz = -sy;
        float speed = 0.08f; // units per frame
        auto pressed = [](sf::Keyboard::Key k){ return sf::Keyboard::isKeyPressed(k); };
        auto scPressed = [](sf::Keyboard::Scancode s){ return sf::Keyboard::isKeyPressed(s); };
        bool w = kW || pressed(sf::Keyboard::Key::W) || scPressed(sf::Keyboard::Scan::W);
        bool s = kS || pressed(sf::Keyboard::Key::S) || scPressed(sf::Keyboard::Scan::S);
        bool d = kD || pressed(sf::Keyboard::Key::D) || scPressed(sf::Keyboard::Scan::D);
        bool a = kA || pressed(sf::Keyboard::Key::A) || scPressed(sf::Keyboard::Scan::A);
        bool e = kE || pressed(sf::Keyboard::Key::E) || scPressed(sf::Keyboard::Scan::E);
        bool q = kQ || pressed(sf::Keyboard::Key::Q) || scPressed(sf::Keyboard::Scan::Q);
        if (w) { camX += fx * speed; camY += fy * speed; camZ += fz * speed; moved = true; }
        if (s) { camX -= fx * speed; camY -= fy * speed; camZ -= fz * speed; moved = true; }
        if (d) { camX += rx * speed; camZ += rz * speed; moved = true; }
        if (a) { camX -= rx * speed; camZ -= rz * speed; moved = true; }
        if (e) { camY += speed; moved = true; }
        if (q) { camY -= speed; moved = true; }
        if (moved) {
            renderer.setCameraPosition(camX, camY, camZ);
            renderRequested = true;
        }
        window.clear();
        if (renderRequested) {
            renderer.startTiledRender(pixels, x, y, z, pool, tileW, tileH);
            renderRequested = false;
        }
        if (renderer.isRenderFinished()) {
            renderRequested = true;
        }
        screenText.update(pixels);
        window.draw(sprite);
        window.display();
    }
}
