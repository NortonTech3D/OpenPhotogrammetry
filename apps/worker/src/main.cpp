#include <iostream>

#include "openphotogrammetry/io/io.hpp"
#include "openphotogrammetry/mvs/mvs.hpp"

int main() {
  const bool ready = op::io::can_read_project_manifest("job.json") && op::mvs::can_dense_reconstruct();
  std::cout << "worker.ready=" << (ready ? "true" : "false") << '\n';
  return ready ? 0 : 1;
}
