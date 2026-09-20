#pragma once

#include "Representation/Implicit.h"
#include "Representation/ImplicitBspline.h"
#include "Representation/Primitive.h"

#include <memory>
#include <variant>

// CSG tree over the primitive implicits. Union/Intersection combine children
// with min/max, which is a conservative bound of the exact SDF, not the SDF
// itself; Negate flips the sign (exact SDF of the complement).
class Csg : public Implicit {
public:
    struct Node;
    using NodePtr = std::unique_ptr<Node>;
    using Primitive = std::variant<Sphere, Plane, ImplicitBspline>;

    struct Union {
        NodePtr lhs, rhs;
    };
    struct Intersection {
        NodePtr lhs, rhs;
    };
    struct Negate {
        NodePtr operand;
    };

    struct Node {
        std::variant<Primitive, Union, Intersection, Negate> value;
    };

    explicit Csg(NodePtr root);

    double Sdf(const Vec3& p) const override;
    Vec3 Grad(const Vec3& p) const override;

    Box3 GetBoundingBox() const override {
        return boundingBox_;
    }

    static NodePtr MakePrimitive(Primitive primitive);
    static NodePtr MakeUnion(NodePtr lhs, NodePtr rhs);
    static NodePtr MakeIntersection(NodePtr lhs, NodePtr rhs);
    static NodePtr MakeNegate(NodePtr operand);

    // Unit sphere with a smaller sphere subtracted: A \ B = A ∩ ¬B.
    static Csg MakeSphereWithSphereCut();

    // Cube built as the intersection of six half-spaces, unioned with a
    // sphere sticking halfway out of its top face.
    static Csg MakeCubeWithSphere();

    // The paca cut in half lengthwise by a plane.
    static Csg MakeHalfPaca();

private:
    NodePtr root_;
    Box3 boundingBox_; // merged bounding boxes of all primitives in the tree
};
