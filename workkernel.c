#include <stdint.h>

// ===================== CCOUNT + WORKKERNEL ======================
static inline uint32_t wk_get_cycle32(void) {
#if defined(__XTENSA__)
  uint32_t cycles;
  asm volatile("rsr %0, ccount" : "=a"(cycles));
  return cycles;
#elif defined(__riscv)
  uint32_t cycles;
  asm volatile("rdcycle %0" : "=r"(cycles));
  return cycles;
#else
#error "Unsupported architecture for WorkKernel cycle counter"
#endif
}
#define COMPILER_BARRIER() asm volatile("" ::: "memory")

static inline uint32_t mix32(uint32_t x) {
  x ^= x >> 16;
  x *= 0x7FEB352Du;
  x ^= x >> 15;
  x *= 0x846CA68Bu;
  x ^= x >> 16;
  return x;
}

// Instructor may replace; in final assignment students must not modify.
uint32_t WorkKernel(uint32_t budget_cycles, uint32_t seed) {
  uint32_t start = wk_get_cycle32();
  uint32_t acc = 0x12345678u ^ seed;
  uint32_t x   = 0x9E3779B9u ^ (seed * 0x85EBCA6Bu);

  while ((uint32_t)(wk_get_cycle32() - start) < budget_cycles) {
    x   = mix32(x + 0x9E3779B9u);
    acc ^= x;
    acc = (acc << 5) | (acc >> 27);
    acc += 0xA5A5A5A5u;
    COMPILER_BARRIER();
  }
  return mix32(acc);
}
