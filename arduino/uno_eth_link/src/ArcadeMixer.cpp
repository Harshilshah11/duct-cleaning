#include "ArcadeMixer.h"

void arcadeMix(int x, int y, int maxPwm, int *left, int *right) {
    // long throughout: y + x reaches 2000, and 2000 * 255 is 510000, which
    // overflows a 16-bit int several times over.
    const long l = (long)y + (long)x;
    const long r = (long)y - (long)x;

    long peak = 1000;                    // == max(1.0, ...) in the Python
    if (labs(l) > peak) peak = labs(l);
    if (labs(r) > peak) peak = labs(r);

    // peak is seeded at 1000 and only ever grows, so it can never be zero —
    // a centred stick divides by 1000, not by 0.
    *left  = (int)(l * (long)maxPwm / peak);
    *right = (int)(r * (long)maxPwm / peak);
}
