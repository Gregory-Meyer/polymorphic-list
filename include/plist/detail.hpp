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

#ifndef PLIST_DETAIL_HPP
#define PLIST_DETAIL_HPP

#include <cassert>
#include <cstddef>

#include <iterator>
#include <memory>
#include <type_traits>
#include <utility>

namespace plist {

template <typename T, typename A = std::allocator<T>>
class PolymorphicList;

namespace detail {

struct NodeLayout {
	NodeLayout *prev;
	NodeLayout *next;
	void (*deleter)(std::allocator<std::max_align_t>&, NodeLayout*);
	std::max_align_t value;
};

template <typename T>
struct NodeBase {
	constexpr NodeBase() noexcept = default;

	NodeBase(const NodeBase &other) = delete;

	NodeBase(NodeBase &&other) = delete;

	NodeBase& operator=(const NodeBase &other) = delete;

	NodeBase& operator=(NodeBase &&other) = delete;

	virtual ~NodeBase() = default;

	T& value() & noexcept {
		return get();
	}

	const T& value() const & noexcept {
		return get();
	}

	T&& value() && noexcept {
		return std::move(get());
	}

	const T&& value() const && noexcept {
		return std::move(get());
	}

	NodeBase *prev = nullptr;
	NodeBase *next = nullptr;

private:
	static constexpr std::size_t offset() noexcept {
		return offsetof(NodeLayout, value);
	}

	constexpr T& get() noexcept {
		return *reinterpret_cast<T*>(reinterpret_cast<char*>(this) + offset());
	}

	constexpr const T& get() const noexcept {
		return *reinterpret_cast<const T*>(reinterpret_cast<const char*>(this) + offset());
	}
};

template <typename T, typename A = std::allocator<T>>
struct NodeBaseWithAllocator : NodeBase<T> {
private:
	using Traits = std::allocator_traits<A>;

	using NodeAllocator =
		typename Traits::template rebind_alloc<NodeBaseWithAllocator>;
	using NodeTraits = typename std::allocator_traits<NodeAllocator>;
	using NodePointer = typename NodeTraits::pointer;

public:
	using Deleter = void (*)(A&, NodePointer);

	NodeBaseWithAllocator(const NodeBaseWithAllocator &other) = delete;

	NodeBaseWithAllocator(NodeBaseWithAllocator &&other) = delete;

	NodeBaseWithAllocator& operator=(
		const NodeBaseWithAllocator &other
	) = delete;

	NodeBaseWithAllocator& operator=(NodeBaseWithAllocator &&other) = delete;

	virtual ~NodeBaseWithAllocator() = default;

	NodeBaseWithAllocator(Deleter deleter) noexcept
	: deleter{ deleter } {
		assert(deleter);
	}

	Deleter deleter;
};

template <typename A, typename T>
struct ReboundAllocType {
	using type = typename std::allocator_traits<A>::template rebind_alloc<T>;
};

template <typename A, typename T>
using ReboundAlloc = typename ReboundAllocType<A, T>::type;

template <typename A, typename T>
struct ReboundTraitsType {
	using type = typename std::allocator_traits<A>::template rebind_traits<T>;
};

template <typename A, typename T>
using ReboundTraits = typename ReboundTraitsType<A, T>::type;

template <typename A, typename T>
struct ReboundPointerType {
	using type = typename ReboundTraits<A, T>::pointer;
};

template <typename A, typename T>
using ReboundPointer = typename ReboundPointerType<A, T>::type;

template <typename A, typename B, typename D>
void deleter(A &alloc, ReboundPointer<A, B> base_ptr) {
	const auto derived_ptr = static_cast<ReboundPointer<A, D>>(base_ptr);

	ReboundAlloc<A, D> derived_alloc{ alloc };
	ReboundTraits<A, D>::destroy(derived_alloc, derived_ptr);
	ReboundTraits<A, D>::deallocate(derived_alloc, derived_ptr, 1);
}

template <typename U, typename T, typename A>
struct Node : NodeBaseWithAllocator<T, A> {
	static_assert(std::is_same<T, U>::value || std::is_base_of<T, U>::value, "");
	static_assert(alignof(U) <= alignof(std::max_align_t), "");

	template <typename ...Ts, std::enable_if_t<std::is_constructible<U, Ts&&...>::value, int> = 0>
	explicit Node(Ts &&...ts)
	noexcept(std::is_nothrow_constructible<U, Ts&&...>::value)
	: NodeBaseWithAllocator<T, A>{ &deleter<A, NodeBaseWithAllocator<T, A>, Node> },
	  value(std::forward<Ts>(ts)...) { }

	Node(const Node &other) = delete;

	Node(Node &&other) = delete;

	Node& operator=(const Node &other) = delete;

	Node& operator=(Node &&other) = delete;

	virtual ~Node() = default;

	alignas(std::max_align_t) U value;
};

template <typename T, typename A = std::allocator<T>>
struct DummyNode : NodeBaseWithAllocator<T, A> {
	constexpr DummyNode() noexcept
	: NodeBaseWithAllocator<T, A>{ &deleter<A, NodeBaseWithAllocator<T, A>, DummyNode> } { }

	constexpr DummyNode(const DummyNode&) noexcept : DummyNode{ } { }

	constexpr DummyNode(DummyNode&&) noexcept : DummyNode{ } { }

	constexpr DummyNode& operator=(const DummyNode&) noexcept {
		return *this;
	}

	constexpr DummyNode& operator=(DummyNode&&) noexcept {
		return *this;
	}
};

template <typename T, bool IsConst>
class Iterator {
public:
	template <typename, typename>
	friend class plist::PolymorphicList;

	template <typename, bool>
	friend class Iterator;

	using difference_type = std::ptrdiff_t;
	using iterator_category = std::bidirectional_iterator_tag;
	using pointer = std::conditional_t<IsConst, const T*, T*>;
	using reference = std::conditional_t<IsConst, const T&, T&>;
	using value_type = T;

	Iterator() noexcept = default;

	template <bool B = IsConst, std::enable_if_t<B, int> = 0>
	Iterator(Iterator<T, !B> other) noexcept
	: current_{ other.current_ } { }

	Iterator& operator++() {
		assert(current_);
		assert(current_->next);

		current_ = current_->next;

		return *this;
	}

	Iterator operator++(int) {
		const auto to_return = *this;

		++*this;

		return to_return;
	}

	Iterator& operator--() {
		assert(current_);
		assert(current_->prev);

		current_ = current_->prev;

		return *this;
	}

	Iterator operator--(int) {
		const auto to_return = *this;

		--*this;

		return to_return;
	}

	reference operator*() const {
		assert(current_);

		return current_->value();
	}

	pointer operator->() const {
		assert(current_);

		return std::addressof(**this);
	}

	friend bool operator==(Iterator lhs, Iterator rhs) noexcept {
		return lhs.current_ == rhs.current_;
	}

	friend bool operator!=(Iterator lhs, Iterator rhs) noexcept {
		return !(lhs == rhs);
	}

private:
	explicit Iterator(NodeBase<T> *current) noexcept : current_{ current } { }

	NodeBase<T> *current_ = nullptr;
};

} // namespace detail
} // namespace plist

#endif
