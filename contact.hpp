#pragma once

#include "glm/ext/quaternion_geometric.hpp"
#include "glm/matrix.hpp"
#include "rigid_body.hpp"
#include <glm/glm.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/quaternion.hpp>

#include <memory>

enum ContactType {
    VERTEX_FACE,
    EDGE_EDGE,
};

struct Contact {
    ContactType contact_type;
    glm::vec3 normal;
    std::shared_ptr<RigidBody> a = nullptr, b = nullptr;

    glm::vec3 p; // only for V-F contact
    glm::vec3 ea, eb; // only for E-E contact

    glm::vec3 get_contact_point();
};

struct Witness {
    glm::vec3 normal;
    glm::vec3 p;
};