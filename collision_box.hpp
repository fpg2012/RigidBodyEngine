#pragma once

#include "bounding_box.hpp"

#include "glm/ext/quaternion_geometric.hpp"
#include "glm/matrix.hpp"
#include <glm/glm.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/quaternion.hpp>

struct CollisionBox {
    BoundingBox3D get_bb(glm::quat &rotation, glm::vec3 translation);
};