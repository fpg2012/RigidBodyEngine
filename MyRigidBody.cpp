#include "ode_solver.hpp"
#include "simulation_engine.hpp"
#include "rigid_body.hpp"
#include <iostream>
#include <memory>

uint64_t id;

std::shared_ptr<Engine> init() {
    auto solver_ref = std::make_shared<ExplicitSolver<float, 13>>();
    auto engine = std::make_shared<Engine>(solver_ref);

    id = engine->add_body(glm::vec3(.0f), glm::quat(1.0f, glm::vec3(.0f)), glm::vec3(.0f), glm::vec3(0.f), glm::mat3(2.0f), 12);
    return engine;
}

int main() {
    auto engine = init();
    Force force{glm::vec3(-0.5f, -0.5f, 0.5f), glm::vec3(0.5f, 0.5f, 0.5f), id};
    engine->add_force(std::move(force));
    for (int i = 0; i < 1000; ++i) {
        std::cout << engine->bodies[id].q.w << ", " << engine->bodies[id].q.x << "," << engine->bodies[id].q.y << "," << engine->bodies[id].q.z << "," << std::endl;
        engine->step();
    }
    return 0;
}