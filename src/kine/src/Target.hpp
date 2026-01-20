#ifndef ROS2_WS_TARGET_HPP
#define ROS2_WS_TARGET_HPP

#include <threepp/geometries/SphereGeometry.hpp>

class Target : public Object3D {
public:
    Target() {
        auto material = MeshBasicMaterial::create();
        material->color = Color::red;

        auto geometry = SphereGeometry::create(1);
        auto mesh = Mesh::create(geometry, material);
        add(mesh);

        update(0);
    }

    void update(float dt) {
        // cirular motion with center c and radius r
        float r = 2.0;
        Vector2 center = {5 ,0};
        position.x = center.x + r * cos(t);
        position.z = center.y + r * sin(t);

        t += dt;
    }

private:
    float t;
};

#endif //ROS2_WS_TARGET_HPP
