#include "../src/mm/PerspectiveWriteGate.hpp"
#include <iostream>
#include <limits>

int main()
{
    utpp::mm::PerspectiveWriteGate gate;
    int failed = 0;
    auto check = [&](bool ok, const char* label) {
        if (!ok) { ++failed; std::cerr << label << '\n'; }
    };
    check(!gate.Allows(100, 50), "startup denies");
    gate.Publish(true, 100);
    check(gate.Allows(100, 50) && gate.Allows(150, 50), "fresh heartbeat allows");
    check(!gate.Allows(151, 50) && !gate.Allows(99, 50), "expired or future heartbeat denies");
    gate.Publish(false, 101);
    check(!gate.Allows(101, 50), "FPP immediately denies despite ongoing MM fade");
    gate.Publish(true, 102);
    check(gate.Allows(102, 50), "new TPP heartbeat allows");
    gate.Reset();
    check(!gate.Allows(102, 50), "disarm invalidates heartbeat");
    gate.Publish(true, -1);
    check(!gate.Allows(103, 50), "invalid clock denies");
    gate.Publish(true, 1);
    check(!gate.Allows(std::numeric_limits<int64_t>::min(), 50), "negative now denies without overflow");
    check(!gate.Allows(std::numeric_limits<int64_t>::max(), 50), "huge age denies without overflow");
    check(!gate.Allows(1, 0), "uninitialized max age denies");
    std::cout << "PerspectiveWriteGate failures=" << failed << '\n';
    return failed ? 1 : 0;
}
