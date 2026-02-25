#include "physkit/collision.h"
#include "test.h"

#include <mp-units/systems/si/unit_symbols.h>

using namespace mp_units;
using namespace mp_units::si::unit_symbols;
using namespace physkit;
using namespace testing;

int main()
{
    // TODO implement GJK tests
    suite s;
    s.group("GJK").test("NOTIMPLEMENTED", [] { throw "Not Implemented"; });
    return s.run();
}
