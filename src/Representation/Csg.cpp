#include "Representation/Csg.h"

#include <algorithm>

namespace {

template<class... Ts>
struct Overloaded : Ts... {
    using Ts::operator()...;
};
template<class... Ts>
Overloaded(Ts...) -> Overloaded<Ts...>;

struct SdfGrad {
    double sdf;
    Vec3 grad;
};

double SdfNode(const Csg::Node& node, const Vec3& p) {
    return std::visit(Overloaded{
                          [&](const Csg::Primitive& primitive) {
                              return std::visit(
                                  [&](const auto& shape) {
                                      return shape.Sdf(p);
                                  },
                                  primitive);
                          },
                          [&](const Csg::Union& u) {
                              return std::min(SdfNode(*u.lhs, p), SdfNode(*u.rhs, p));
                          },
                          [&](const Csg::Intersection& i) {
                              return std::max(SdfNode(*i.lhs, p), SdfNode(*i.rhs, p));
                          },
                          [&](const Csg::Negate& n) {
                              return -SdfNode(*n.operand, p);
                          },
                      },
                      node.value);
}

// Sdf and Grad together, so the min/max branch is selected once per node.
SdfGrad EvalNode(const Csg::Node& node, const Vec3& p) {
    return std::visit(Overloaded{
                          [&](const Csg::Primitive& primitive) {
                              return std::visit(
                                  [&](const auto& shape) {
                                      return SdfGrad{shape.Sdf(p), shape.Grad(p)};
                                  },
                                  primitive);
                          },
                          [&](const Csg::Union& u) {
                              const SdfGrad lhs = EvalNode(*u.lhs, p);
                              const SdfGrad rhs = EvalNode(*u.rhs, p);
                              return lhs.sdf <= rhs.sdf ? lhs : rhs;
                          },
                          [&](const Csg::Intersection& i) {
                              const SdfGrad lhs = EvalNode(*i.lhs, p);
                              const SdfGrad rhs = EvalNode(*i.rhs, p);
                              return lhs.sdf >= rhs.sdf ? lhs : rhs;
                          },
                          [&](const Csg::Negate& n) {
                              SdfGrad result = EvalNode(*n.operand, p);
                              result.sdf = -result.sdf;
                              result.grad = -result.grad;
                              return result;
                          },
                      },
                      node.value);
}

void MergePrimitiveBoxes(const Csg::Node& node, Box3& box) {
    std::visit(Overloaded{
                   [&](const Csg::Primitive& primitive) {
                       box.extend(std::visit(
                           [](const auto& shape) {
                               return shape.GetBoundingBox();
                           },
                           primitive));
                   },
                   [&](const Csg::Union& u) {
                       MergePrimitiveBoxes(*u.lhs, box);
                       MergePrimitiveBoxes(*u.rhs, box);
                   },
                   [&](const Csg::Intersection& i) {
                       MergePrimitiveBoxes(*i.lhs, box);
                       MergePrimitiveBoxes(*i.rhs, box);
                   },
                   [&](const Csg::Negate& n) {
                       MergePrimitiveBoxes(*n.operand, box);
                   },
               },
               node.value);
}

} // namespace

Csg::Csg(NodePtr root)
: root_(std::move(root)) {
    boundingBox_.setEmpty();
    MergePrimitiveBoxes(*root_, boundingBox_);
}

double Csg::Sdf(const Vec3& p) const {
    return SdfNode(*root_, p);
}

Vec3 Csg::Grad(const Vec3& p) const {
    return EvalNode(*root_, p).grad;
}

Csg::NodePtr Csg::MakePrimitive(Primitive primitive) {
    return std::make_unique<Node>(Node{std::move(primitive)});
}

Csg::NodePtr Csg::MakeUnion(NodePtr lhs, NodePtr rhs) {
    return std::make_unique<Node>(Node{Union{std::move(lhs), std::move(rhs)}});
}

Csg::NodePtr Csg::MakeIntersection(NodePtr lhs, NodePtr rhs) {
    return std::make_unique<Node>(Node{Intersection{std::move(lhs), std::move(rhs)}});
}

Csg::NodePtr Csg::MakeNegate(NodePtr operand) {
    return std::make_unique<Node>(Node{Negate{std::move(operand)}});
}

Csg Csg::MakeSphereWithSphereCut() {
    NodePtr body = MakePrimitive(Sphere(Vec3::Zero(), 1.0));
    NodePtr cut = MakePrimitive(Sphere(Vec3(0.6, 0.6, 0.6), 0.7));
    return Csg(MakeIntersection(std::move(body), MakeNegate(std::move(cut))));
}

Csg Csg::MakeCubeWithSphere() {
    constexpr double kHalfSize = 0.6;
    NodePtr cube;
    for (int axis = 0; axis < 3; ++axis) {
        for (double sign : {-1.0, 1.0}) {
            NodePtr face = MakePrimitive(Plane(sign * Vec3::Unit(axis), kHalfSize));
            cube = cube ? MakeIntersection(std::move(cube), std::move(face)) : std::move(face);
        }
    }
    NodePtr bump = MakePrimitive(Sphere(Vec3(0.0, kHalfSize, 0.0), 0.4));
    return Csg(MakeUnion(std::move(cube), std::move(bump)));
}

Csg Csg::MakeHalfPaca() {
    NodePtr paca = MakePrimitive(ImplicitBspline::MakePaca());
    // The paca lies along the X axis, so the z = 0 plane cuts it lengthwise.
    NodePtr cut = MakePrimitive(Plane(Vec3::UnitZ(), 0.0));
    return Csg(MakeIntersection(std::move(paca), std::move(cut)));
}
