#include <stdio.h>
#include <mdsplus.hpp>
using namespace mdsplus;
int main() {
    auto tree = Tree("josh", 1, Mode::Normal); 
    auto node = tree.getNode("DUMMY");

    std::vector<uint8_t> bytes;
    bytes.resize(600 * 400);

    node.makeSegment(0., 1E-6, 0., bytes);
    node.makeSegment(1., 1+1E-6, 1., bytes);


}
