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
	virtual ~NodeBase() = default;

	virtual T& value() & noexcept = 0;

	virtual const T& value() const & noexcept = 0;

	virtual T&& value() && noexcept = 0;

	virtual const T&& value() const && noexcept = 0;

	NodeBase *prev = nullptr;
	NodeBase *next = nullptr;
};

template <typename T, typename A = std::allocator<T>>
struct NodeBaseWithAllocator : NodeBase<T> {
private:
	using Traits = std::allocator_traits<A>;
	using NodeAllocator =
		typename Traits::template rebind_alloc<NodeBaseWithAllocator>;
	using NodeTraits = typename std::allocator_traits<NodeAllocator>;
	using Pointer = typename NodeTraits::pointer;

public:
	using Deleter = void (*)(A&, Pointer);

	virtual ~NodeBaseWithAllocator() = default;

	virtual Deleter get_deleter() = 0;
};

template <typename U, typename T, typename A = std::allocator<T>,
		  std::enable_if_t<std::is_same<T, U>::value || std::is_base_of<T, U>::value, int> = 0>
struct Node : NodeBaseWithAllocator<T, A> {
	template <
		typename ...Ts,
		std::enable_if_t<std::is_constructible<U, Ts&&...>::value, int> = 0
	>
	explicit Node(Ts &&...ts)
	noexcept(std::is_nothrow_constructible<U, Ts&&...>::value)
	: value_(std::forward<Ts>(ts)...) { }

	virtual ~Node() = default;

	T& value() & noexcept override {
		return value_;
	}

	const T& value() const & noexcept override {
		return value_;
	}

	T&& value() && noexcept override {
		return std::move(value_);
	}

	const T&& value() const && noexcept override {
		return std::move(value_);
	}

	typename NodeBaseWithAllocator<T, A>::Deleter get_deleter() override {
		return &Node::deleter;
	}

private:
	using Traits = std::allocator_traits<A>;
	using BaseAllocator =
		typename Traits::template rebind_alloc<NodeBaseWithAllocator<T, A>>;
	using BaseTraits = typename std::allocator_traits<BaseAllocator>;
	using BasePointer = typename BaseTraits::pointer;

	using NodeAllocator = typename Traits::template rebind_alloc<Node>;
	using NodeTraits = typename std::allocator_traits<NodeAllocator>;
	using NodePointer = typename NodeTraits::pointer;

	static void deleter(A &alloc, BasePointer base_ptr) {
		assert(base_ptr);

		const auto node_ptr = static_cast<NodePointer>(base_ptr);

		NodeAllocator node_alloc{ alloc };
		NodeTraits::destroy(node_alloc, node_ptr);
		NodeTraits::deallocate(node_alloc, node_ptr, 1);
	}

	U value_;
};

template <typename T, bool IsConst>
class Iterator {
public:
	template <typename U, typename A>
	friend class plist::PolymorphicList;

	using difference_type = std::ptrdiff_t;
	using iterator_category = std::bidirectional_iterator_tag;
	using pointer = std::conditional_t<IsConst, const T*, T*>;
	using reference = std::conditional_t<IsConst, const T&, T&>;
	using value_type = T;

	Iterator() noexcept = default;

	Iterator(Iterator<T, false> other) noexcept
	: current_{ other.current_ }, is_past_end_{ other.is_past_end_ } { }

	Iterator& operator++() {
		assert(current_);
		assert(!is_past_end_);

		if (!current_->next) {
			is_past_end_ = true;
		} else {
			current_ = current_->next;
		}

		return *this;
	}

	Iterator operator++(int) {
		const auto to_return = *this;

		++*this;

		return to_return;
	}

	Iterator& operator--() {
		assert(current_);
		assert(is_past_end_ || current_->prev);

		if (is_past_end_) {
			is_past_end_ = false;
		} else {
			current_ = current_->prev;
		}

		return *this;
	}

	Iterator operator--(int) {
		const auto to_return = *this;

		--*this;

		return to_return;
	}

	reference operator*() const {
		assert(current_);
		assert(!is_past_end_);

		return current_->value();
	}

	pointer operator->() const {
		assert(current_);
		assert(!is_past_end_);

		return std::addressof(**this);
	}

	friend bool operator==(Iterator lhs, Iterator rhs) noexcept {
		if (lhs.is_past_end_ != rhs.is_past_end_) {
			return false;
		} else if (lhs.is_past_end_ && rhs.is_past_end_) {
			return true;
		}

		return lhs.current_ == rhs.current_;
	}

	friend bool operator!=(Iterator lhs, Iterator rhs) noexcept {
		return !(lhs == rhs);
	}

private:
	explicit Iterator(NodeBase<T> *current) noexcept
	: Iterator{ current, false } { }

	Iterator(NodeBase<T> *current, bool is_past_end) noexcept
	: current_{ current }, is_past_end_{ is_past_end } { }

	NodeBase<T> *current_ = nullptr;
	bool is_past_end_ = true;
};

} // namespace detail
} // namespace plist

#endif
