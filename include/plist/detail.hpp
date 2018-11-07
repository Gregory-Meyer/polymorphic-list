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

#include <iterator>
#include <memory>
#include <type_traits>
#include <utility>

namespace plist {

template <typename T, typename A = std::allocator<T>>
class PolymorphicList;

namespace detail {

template <typename T>
struct NodeBase {
	NodeBase(const NodeBase &other) = delete;

	NodeBase(NodeBase &&other) = delete;

	NodeBase& operator=(const NodeBase &other) = delete;

	NodeBase& operator=(NodeBase &&other) = delete;

	virtual ~NodeBase() = default;

	T& value() & noexcept {
		assert(value_ptr_);

		return *value_ptr_;
	}

	const T& value() const & noexcept {
		assert(value_ptr_);

		return *value_ptr_;
	}

	T&& value() && noexcept {
		assert(value_ptr_);

		return std::move(*value_ptr_);
	}

	const T&& value() const && noexcept {
		assert(value_ptr_);

		return std::move(*value_ptr_);
	}

	NodeBase *prev = nullptr;
	NodeBase *next = nullptr;

protected:
	NodeBase(T *value_ptr) noexcept : value_ptr_{ value_ptr } { }

private:
	T *value_ptr_;
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

	Deleter get_deleter() const noexcept {
		assert(deleter_);

		return deleter_;
	}

protected:
	NodeBaseWithAllocator(T *value_ptr, Deleter deleter) noexcept
	: NodeBase<T>{ value_ptr }, deleter_{ deleter } {
		assert(deleter);
	}

	Deleter deleter_;
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

template <typename U, typename T, typename A = std::allocator<T>,
		  std::enable_if_t<std::is_same<T, U>::value || std::is_base_of<T, U>::value, int> = 0>
struct Node : NodeBaseWithAllocator<T, A> {
	template <
		typename ...Ts,
		std::enable_if_t<std::is_constructible<U, Ts&&...>::value, int> = 0
	>
	explicit Node(Ts &&...ts)
	noexcept(std::is_nothrow_constructible<U, Ts&&...>::value) :
		NodeBaseWithAllocator<T, A>{
			std::addressof(value_),
			&deleter<A, NodeBaseWithAllocator<T, A>, Node>
		},
		value_(std::forward<Ts>(ts)...)
	{ }

	Node(const Node &other) = delete;

	Node(Node &&other) = delete;

	Node& operator=(const Node &other) = delete;

	Node& operator=(Node &&other) = delete;

	virtual ~Node() = default;

private:
	U value_;
};

template <typename T, typename A = std::allocator<T>>
struct DummyNode : NodeBaseWithAllocator<T, A> {
	constexpr DummyNode() noexcept : NodeBaseWithAllocator<T, A>{
		nullptr,
		&deleter<A, NodeBaseWithAllocator<T, A>, DummyNode>
	} { }
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
