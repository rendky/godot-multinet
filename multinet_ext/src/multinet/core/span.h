#ifndef MULTINET_SPAN_H
#define MULTINET_SPAN_H

#include <cstddef>

namespace Multinet {

template <typename T>
class Span {
private:
	T *ptr{ nullptr };
	size_t len{ 0 };

public:
	constexpr Span() = default;

	constexpr Span(T *p_ptr, size_t p_len)
		: ptr(p_ptr), len(p_len) {}

	template <size_t N>
	constexpr Span(T (&p_arr)[N])
		: ptr(p_arr), len(N) {}

	[[nodiscard]] constexpr T *data() const noexcept { return ptr; }
	[[nodiscard]] constexpr size_t size() const noexcept { return len; }
	[[nodiscard]] constexpr bool empty() const noexcept { return len == 0; }

	[[nodiscard]] constexpr T &operator[](size_t p_idx) const noexcept { return ptr[p_idx]; }

	[[nodiscard]] constexpr T *begin() const noexcept { return ptr; }
	[[nodiscard]] constexpr T *end() const noexcept { return ptr + len; }
};

template <typename T>
class Span<const T> {
private:
	const T *ptr{ nullptr };
	size_t len{ 0 };

public:
	constexpr Span() = default;

	constexpr Span(const T *p_ptr, size_t p_len)
		: ptr(p_ptr), len(p_len) {}

	template <size_t N>
	constexpr Span(const T (&p_arr)[N])
		: ptr(p_arr), len(N) {}

	constexpr Span(Span<T> p_non_const)
		: ptr(p_non_const.data()), len(p_non_const.size()) {}

	[[nodiscard]] constexpr const T *data() const noexcept { return ptr; }
	[[nodiscard]] constexpr size_t size() const noexcept { return len; }
	[[nodiscard]] constexpr bool empty() const noexcept { return len == 0; }

	[[nodiscard]] constexpr const T &operator[](size_t p_idx) const noexcept { return ptr[p_idx]; }

	[[nodiscard]] constexpr const T *begin() const noexcept { return ptr; }
	[[nodiscard]] constexpr const T *end() const noexcept { return ptr + len; }
};

} // namespace Multinet

#endif // MULTINET_SPAN_H
