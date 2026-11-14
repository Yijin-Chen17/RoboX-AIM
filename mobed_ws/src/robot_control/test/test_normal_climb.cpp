#include <iostream>
#include <stdexcept>
#include "robot_control/tasks/task_controller.hpp"

using namespace robot_control;
using tasks::TaskController;
using tasks::NormalClimbController;

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

struct Contact final : estimation::ContactDetector {
    bool front = false;
    bool rear = false;
    bool frontImpactDetected() const override { return front; }
    bool rearImpactDetected() const override { return rear; }
};

int main() {
    try {
        TaskController task;
        Contact contact;
        estimation::ClimbGeometry geometry;
        Eigen::Vector4d q(.813, .813, -.813, -.813);
        const auto tick = [&](double speed = .35, double requested = .25) {
            task.update(contact, geometry, q, Eigen::Vector4d::Constant(4), .01, speed, requested);
            if (task.isActive()) q = task.getEccOverrides();
        };
        for (int n = 0; n < 100; ++n) tick();
        require(!task.isActive(), "flat driving triggered normal climb");
        contact.front = true;
        task.setNormalApproachAllowed(false);
        tick();
        require(!task.isActive(), "strafe/turn approach started normal climb");
        task.setNormalApproachAllowed(true);
        tick();
        require(int(task.getState()) == 20, "normal contact did not start independent front lift");
        geometry.delta_height = .04;
        for (int n = 0; n < 200 && int(task.getState()) == 20; ++n) tick();
        require(int(task.getState()) == 21, "front landing did not advance normal FSM");
        for (int n = 0; n < 100; ++n) tick();
        require(int(task.getState()) == 21, "rear lifted before arrival evidence");
        contact.rear = true;
        for (int n = 0; n < 20 && int(task.getState()) == 21; ++n) tick(.05);
        require(int(task.getState()) == 22, "rear contact did not start normal lift");
        geometry.delta_height = 0;
        for (int n = 0; n < 900 && task.isActive(); ++n) tick();
        require(!task.isActive() && task.hasClimbed(), "normal climb did not finish and hand back control");

        task.reset();
        contact.front = contact.rear = false;
        q << .813, .813, -.813, -.813;
        geometry.delta_height = .04;
        for (int n = 0; n < 200; ++n) tick();
        require(!task.isActive(), "height difference without obstruction triggered climbing");
        // A curb resistance episode followed by free rolling: the front axle
        // already made it up, so restoring the historical branch must skip lift.
        task.setNormalApproachAllowed(false);
        contact.front = true;
        tick();
        contact.front = false;
        task.setNormalApproachAllowed(true);
        for (int n = 0; n < 20 && !task.isActive(); ++n) tick();
        require(int(task.getState()) == 21, "already-on-curb branch repeated the front lift");

        // Switching modes discards commands, timers and blend-out in both directions.
        task.setClimbMode(ClimbMode::STAIR_200MM);
        require(!task.isActive() && task.getBlendWeight() == 0 && !task.hasClimbed(),
                "normal state leaked into stair mode");
        q.setZero();
        contact.front = true;
        tick();
        require(task.getState() == tasks::ClimbState::FRONT_WALL_CLIMB,
                "200 mm mode failed to use its original FSM");
        task.setClimbMode(ClimbMode::PAPER_NORMAL);
        require(!task.isActive() && task.getBlendWeight() == 0,
                "stair state leaked into normal mode");
        require(task.getEccOverrides().isZero(), "old stair override survived switch");
        contact.front = false;
        tick();
        require(!task.isActive(), "normal resumed a previous climb after switch");
        q << .813, .813, -.813, -.813;
        contact.front = true;
        tick();
        require(task.isActive(), "normal did not re-arm after switch");
        tick(.0, .0);
        require(!task.isActive(), "operator stop failed to cancel normal climb");
        for (int n = 0; n < 100; ++n) tick(.0, .25);
        require(!task.isActive(), "failed climb retriggered while forward command remained held");
        task.setClimbMode(ClimbMode::DISABLED);
        for (int n = 0; n < 20; ++n) tick();
        require(!task.isActive() && task.getBlendWeight() == 0, "disabled mode retained a climb");
        std::cout << "Normal climb sequence and mode isolation passed\n";
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
