// Offline extraction using the installed simulator mesh loader. No Gazebo/ROS node.
#include <ignition/common/MeshManager.hh>
#include <ignition/common/Mesh.hh>
#include <ignition/common/SubMesh.hh>
#include <iomanip>
#include <iostream>
int main(int argc, char **argv) {
  if (argc != 2) return 2;
  const auto *mesh = ignition::common::MeshManager::Instance()->Load(argv[1]);
  if (!mesh) return 3;
  std::cout << std::setprecision(17) << '[';
  bool first = true;
  for (unsigned i = 0; i < mesh->SubMeshCount(); ++i) {
    auto sub = mesh->SubMeshByIndex(i).lock();
    if (!sub || sub->SubMeshPrimitiveType() != ignition::common::SubMesh::TRIANGLES
        || sub->IndexCount() % 3) return 4;
    for (unsigned j = 0; j < sub->IndexCount(); j += 3) {
      if (!first) std::cout << ',';
      first = false;
      std::cout << '[';
      for (unsigned k = 0; k < 3; ++k) {
        const int index = sub->Index(j+k);
        if (index < 0 || static_cast<unsigned>(index) >= sub->VertexCount()) return 5;
        auto v = sub->Vertex(index);
        if (k) std::cout << ',';
        std::cout << '[' << v.X() << ',' << v.Y() << ',' << v.Z() << ']';
      }
      std::cout << ']';
    }
  }
  std::cout << "]\n";
}
