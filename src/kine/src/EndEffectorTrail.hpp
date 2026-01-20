#ifndef ROS2_WS_ENDEFFECTORTRAIL_HPP
#define ROS2_WS_ENDEFFECTORTRAIL_HPP

#include <threepp/objects/Line.hpp>
#include <threepp/core/Object3D.hpp>

class EndEffectorTrail : public Object3D {
public:
    EndEffectorTrail() {
        const auto material = MeshBasicMaterial::create();
        material->color = Color::blue;

        geometry_ = BufferGeometry::create();
        geometry_->setAttribute(
            "position",
            FloatBufferAttribute::create(std::vector<float>(maxPoints_ * 3), 3));
        geometry_->setDrawRange(0, 0);

        auto line = Line::create(geometry_, material);
        line->frustumCulled = false;
        this->add(line);
    }

    void update(const Vector3 &p) {
        const auto pos = geometry_->getAttribute<float>("position");
        auto &a = pos->array();

        if (count_ < maxPoints_) {
            const int i = count_ * 3;
            a[i + 0] = p.x;
            a[i + 1] = p.y;
            a[i + 2] = p.z;
            count_++;
        } else {
            std::memmove(a.data(), a.data() + 3,
                         (maxPoints_ - 1) * 3 * sizeof(float));

            const int i = (maxPoints_ - 1) * 3;
            a[i + 0] = p.x;
            a[i + 1] = p.y;
            a[i + 2] = p.z;
        }

        geometry_->setDrawRange(0, count_);

        pos->needsUpdate();
    }

private:
    int count_ = 0;
    int maxPoints_ = 1000;
    std::shared_ptr<BufferGeometry> geometry_;
};

#endif //ROS2_WS_ENDEFFECTORTRAIL_HPP
