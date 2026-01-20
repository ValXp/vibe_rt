#include "app/CameraController.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

using app::CameraController;
using app::Key;
using app::MouseButton;

TEST_CASE("CameraController updates plane on arrow keys") {
    CameraController controller(0.0f, 0.0f, 1.0f);
    controller.consumeRenderRequest();
    controller.handleKeyReleased(Key::Left);
    auto state = controller.state();
    CHECK(state.planeX == Catch::Approx(-0.1f));
    CHECK(controller.consumeRenderRequest());
}

TEST_CASE("CameraController moves with polled keys") {
    CameraController controller(0.0f, 0.0f, 1.0f);
    controller.setCameraPosition(0.0f, 0.0f, 0.0f);
    controller.setPolledKeys(true, false, false, false, false, false);
    controller.update();
    auto state = controller.state();
    CHECK(state.camZ == Catch::Approx(0.08f));
}

TEST_CASE("CameraController handles mouse rotation") {
    CameraController controller(0.0f, 0.0f, 1.0f);
    controller.handleMousePressed(MouseButton::Left, 10, 10);
    controller.handleMouseMoved(20, 5, true);
    auto state = controller.state();
    CHECK(state.yaw != Catch::Approx(0.0f));
    CHECK(state.pitch != Catch::Approx(0.0f));
}
