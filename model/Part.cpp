#include "Part.h"
#include "utils/Logger.h"

namespace model
{

Part::Part(const std::string& n,
           const Matrix3& I,
           double m,
           const Vector3& centerMass)
   : parent(nullptr),
     name(n),
     inertiaTensor(I),
     compositeInertiaTensor(I),
     mass(m),
     compositeMass(m),
     cm(centerMass),
     needsRecomputing(false),
     childParts()
{ }

Part::~Part()
{}

Part::Part(const Part& orig)
   : parent(orig.parent),
     name(orig.name),
     inertiaTensor(orig.inertiaTensor),
     compositeInertiaTensor(orig.compositeInertiaTensor),
     mass(orig.mass),
     compositeMass(orig.compositeMass),
     cm(orig.cm),
     needsRecomputing(orig.needsRecomputing),
     childParts()
{

   // We are copying the whole tree. If the part we're copying itself has child
   // parts, we are also copying all of them! This may be inefficient and not what
   // is desired, but it is less likely to lead to weird bugs with the same part
   // appearing in multiple locations of the rocket
   utils::Logger::getInstance()->debug("Calling model::Part copy constructor. Recursively copying all child parts. Check Part names for uniqueness");
   
   
   for(const auto& i : orig.childParts)
   {
      Part& x = *std::get<0>(i);
      std::shared_ptr<Part> tempPart = std::make_shared<Part>(x);
      childParts.emplace_back(tempPart, std::get<1>(i));;
   }

}

double Part::getChildMasses(double t)
{
   double childMasses{0.0};
   for(const auto& i : childParts)
   {
      childMasses += std::get<0>(i)->getMass(t);
   }
   return childMasses;

}

void Part::addChildPart(const Part& childPart, Vector3 position)
{
   
   double childMass = childPart.compositeMass;
   Matrix3 childInertiaTensor = childPart.compositeInertiaTensor;
   std::shared_ptr<Part> newChild = std::make_shared<Part>(childPart);
   // Set the parent pointer
   newChild->parent = this;

   // Recompute inertia tensor

   childInertiaTensor += childMass * ( position.dot(position) * Matrix3::Identity() - position*position.transpose());

   compositeInertiaTensor += childInertiaTensor;
   compositeMass += childMass;

   childParts.emplace_back(std::move(newChild), std::move(position));

   if(parent)
   {
      parent->markAsNeedsRecomputing();
      parent->recomputeInertiaTensor();
   }

}

void Part::recomputeInertiaTensor()
{
   if(!needsRecomputing)
   {
      return;
   }
   // recompute the whole composite inertia tensor
   // Reset the composite inertia tensor
   compositeInertiaTensor = inertiaTensor;
   compositeMass = mass;
   for(auto& [child, pos] : childParts)
   {
      child->recomputeInertiaTensor();
      compositeInertiaTensor += child->compositeInertiaTensor + child->compositeMass * ( pos.dot(pos) * Matrix3::Identity() - pos*pos.transpose());
      compositeMass += child->compositeMass;
   }
   needsRecomputing = false;
}

} // namespace model
