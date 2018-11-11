// BSD 3-Clause License
//
// Copyright (c) 2018, Gregory Meyer
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <functional>
#include <initializer_list>
#include <iterator>
#include <type_traits>
#include <utility>

template <typename ...>
using Void = void;

template <typename I, typename = void>
struct IsIterator : std::false_type { };

template <typename I>
struct IsIterator<I, Void<
	typename std::iterator_traits<I>::difference_type,
	typename std::iterator_traits<I>::value_type,
	typename std::iterator_traits<I>::pointer,
	typename std::iterator_traits<I>::reference,
	typename std::iterator_traits<I>::iterator_category
>> : std::true_type { };

template <typename I, bool = IsIterator<I>::value>
struct IterValueType { };

template <typename I>
struct IterValueType<I, true> {
	using type = typename std::iterator_traits<I>::value_type;
};

template <typename I>
using IterValue = typename IterValueType<I>::type;

template <typename I, bool = IsIterator<I>::value>
struct IterRefType { };

template <typename I>
struct IterRefType<I, true> {
	using type = typename std::iterator_traits<I>::reference;
};

template <typename I>
using IterRef = typename IterRefType<I>::type;

namespace {

using std::begin;
using std::end;

template <typename R, typename = void>
struct IsRange : std::false_type { };

template <typename R>
struct IsRange<R, Void<
	decltype(begin(std::declval<R>())),
	decltype(end(std::declval<R>())),
	std::enable_if_t<std::is_same<
		decltype(begin(std::declval<R>())),
		decltype(end(std::declval<R>()))
	>::value && IsIterator<
		decltype(begin(std::declval<R>()))
	>::value>
>> : std::true_type { };

template <typename R, typename = void>
struct RangeIterType { };

template <typename R>
struct RangeIterType<R, Void<std::enable_if_t<IsRange<R>::value>>> {
	using type = decltype(begin(std::declval<R>()));
};

template <typename R>
using RangeIter = typename RangeIterType<R>::type;

template <typename R, typename = decltype(begin(std::declval<R&&>()))>
constexpr auto adl_begin(R &&range) noexcept {
	return begin(std::forward<R>(range));
}

template <typename R, typename = decltype(end(std::declval<R&&>()))>
constexpr auto adl_end(R &&range) noexcept {
	return end(std::forward<R>(range));
}

} // namespace

template <typename I>
class Range {
public:
	template <
		typename R,
		std::enable_if_t<
			IsRange<R&&>::value
			&& std::is_convertible<RangeIter<R&&>, I>::value,
			int
		> = 0
	>
	constexpr Range(R &&range) noexcept
	: first_(adl_begin(std::forward<R>(range))),
	  last_(adl_end(std::forward<R>(range))) { }


	constexpr Range(I first, I last) noexcept
	: first_{ first }, last_{ last } { }

	I begin() const noexcept {
		return first_;
	}

	I end() const noexcept {
		return last_;
	}

private:
	I first_;
	I last_;
};

template <typename R, std::enable_if_t<IsRange<R&&>::value, int> = 0>
constexpr Range<RangeIter<R&&>> make_range(R &&range) noexcept {
	using std::begin;
	using std::end;

	return { begin(std::forward<R>(range)), end(std::forward<R>(range)) };
}

template <typename I, typename J>
constexpr bool operator==(const Range<I> &lhs, const Range<J> &rhs) {
	using std::begin;
	using std::end;

	return std::equal(begin(lhs), end(lhs), begin(rhs), end(rhs));
}

template <typename I, typename J>
constexpr bool operator!=(const Range<I> &lhs, const Range<J> &rhs) {
	return !(lhs == rhs);
}

template <typename I, typename R,
		  std::enable_if_t<IsRange<R&&>::value, int> = 0>
constexpr bool operator==(const Range<I> &lhs, const R &rhs) {
	return lhs == make_range(rhs);
}

template <typename I, typename R,
		  std::enable_if_t<IsRange<R&&>::value, int> = 0>
constexpr bool operator!=(const Range<I> &lhs, const R &rhs) {
	return !(lhs == rhs);
}

template <typename R, typename I,
		  std::enable_if_t<IsRange<R&&>::value, int> = 0>
constexpr bool operator==(const R &lhs, const Range<I> &rhs) {
	return make_range(lhs) == rhs;
}

template <typename R, typename I,
		  std::enable_if_t<IsRange<R&&>::value, int> = 0>
constexpr bool operator!=(const R &lhs, const Range<I> &rhs) {
	return !(lhs == rhs);
}

template <typename R, std::enable_if_t<IsRange<R&&>::value, int> = 0>
Range<std::reverse_iterator<RangeIter<R&&>>> reverse(R &&range) noexcept {
	using std::begin;
	using std::end;

	const auto first = std::make_reverse_iterator(end(std::forward<R>(range)));
	const auto last = std::make_reverse_iterator(begin(std::forward<R>(range)));

	return { first, last };
}

template <typename T>
constexpr std::initializer_list<T> make_ilist(std::initializer_list<T> list) noexcept {
	return list;
}

template <typename ...Ts>
constexpr auto make_ilist(Ts &&...ts) noexcept {
	return make_ilist({ std::forward<Ts>(ts)... });
}

namespace detail {

template <typename C, typename = void, typename ...As>
struct IsMemberPtrInvocable : std::false_type { };

template <typename C, typename ...As>
struct IsMemberPtrInvocable<C, Void<
	decltype(std::mem_fn(std::declval<C>())(std::declval<As>()...))
>, As...> : std::true_type { };

template <typename C, typename = void, typename ...As>
struct IsRegularInvocable : std::false_type { };

template <typename C, typename ...As>
struct IsRegularInvocable<C, Void<
	decltype(std::declval<C>()(std::declval<As>()...))
>, As...> : std::true_type { };

} // namespace detail

template <typename C, typename = void, typename ...As>
struct IsInvocable : std::integral_constant<bool,
	detail::IsRegularInvocable<C, void, As...>::value
	|| detail::IsMemberPtrInvocable<C, void, As...>::value
> { };

template <typename C, typename ...As,
         std::enable_if_t<detail::IsMemberPtrInvocable<C, void, As...>::value, int> = 0>
constexpr decltype(auto) invoke(C &&callable, As &&...args)
noexcept(noexcept(std::mem_fn(std::forward<C>(callable))(std::forward<As>(args)...))) {
    return std::mem_fn(std::forward<C>(callable))(std::forward<As>(args)...);
}

template <typename C, typename ...As,
         std::enable_if_t<detail::IsRegularInvocable<C, void, As...>::value, int> = 0>
constexpr decltype(auto) invoke(C &&callable, As &&...args)
noexcept(noexcept(std::forward<C>(callable)(std::forward<As>(args)...))) {
    return std::forward<C>(callable)(std::forward<As>(args)...);
}
