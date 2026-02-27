#ifndef UTILS_H
#define UTILS_H

#define MAX(x, y) ((x) > (y) ? (x) : (y))
#define MIN(x, y) ((x) > (y) ? (y) : (x))
// Integer division rounding up.
#define DIV_UP(num, denom) ((num + denom - 1) / denom)
// Sets bits in a register <reg> masked by <mask> to value <value>.
#define SET_REG(reg, mask, value) (reg = (reg & ~(mask)) | value)

#endif // UTILS_H
