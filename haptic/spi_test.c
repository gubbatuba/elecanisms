#include <p24FJ128GB206.h>
#include "config.h"
#include "common.h"
#include "ui.h"
#include "pin.h"
#include "spi.h"

int main(void) {
    int test = 0;
    test = 2 + 4; // Tests to see whether scons builds were working.

    init_spi();

}