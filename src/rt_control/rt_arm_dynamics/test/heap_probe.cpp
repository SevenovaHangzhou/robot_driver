// Test-only glibc allocation interposition. Unlike a caller-only Eigen macro,
// this also observes heap calls made inside the distro Pinocchio shared
// library.
#include <cstddef>

namespace
{
thread_local bool observing = false;
thread_local std::size_t allocations = 0U;
} // namespace

extern "C" {
void * __libc_malloc(std::size_t);
void * __libc_calloc(std::size_t, std::size_t);
void * __libc_realloc(void *, std::size_t);
void * __libc_memalign(std::size_t, std::size_t);

void * malloc(std::size_t size) noexcept
{
  if (observing) {
    ++allocations;
  }
  return __libc_malloc(size);
}
void * calloc(std::size_t count, std::size_t size) noexcept
{
  if (observing) {
    ++allocations;
  }
  return __libc_calloc(count, size);
}
void * realloc(void * pointer, std::size_t size) noexcept
{
  if (observing) {
    ++allocations;
  }
  return __libc_realloc(pointer, size);
}
void * aligned_alloc(std::size_t alignment, std::size_t size) noexcept
{
  if (observing) {
    ++allocations;
  }
  return __libc_memalign(alignment, size);
}
int posix_memalign(
  void ** pointer, std::size_t alignment,
  std::size_t size) noexcept
{
  if (alignment < sizeof(void *) || (alignment & (alignment - 1U)) != 0U) {
    return 22;
  }
  if (observing) {
    ++allocations;
  }
  void * result = __libc_memalign(alignment, size);
  if (result == nullptr) {
    return 12;
  }
  *pointer = result;
  return 0;
}
}

void begin_heap_probe() noexcept
{
  allocations = 0U;
  observing = true;
}
std::size_t end_heap_probe() noexcept
{
  observing = false;
  return allocations;
}
