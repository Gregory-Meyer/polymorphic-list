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
struct IdentityType {
	using type = T;
};

template <typename T>
using Identity = typename IdentityType<T>::type;

template <typename T, typename A>
struct NodeBase {
protected:
	using Traits = std::allocator_traits<A>;

public:
	using Pointer = typename Traits::pointer;
	using Difference = typename Traits::difference_type;

	NodeBase(Pointer prev, Pointer next) noexcept
	: prev{ prev }, next{ next } { }

	virtual ~NodeBase() = default;

	virtual T& value() & noexcept = 0;

	virtual const T& value() const & noexcept = 0;

	virtual T&& value() && noexcept = 0;

	virtual const T&& value() const && noexcept = 0;

	virtual void deallocate(A &alloc);

	Pointer prev;
	Pointer next;
};

template <typename U, typename T, typename A,
		  std::enable_if_t<std::is_base_of<T, U>::value, int> = 0>
class Node : public NodeBase<T, A> {
public:
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

	void deallocate(A &alloc) override {
		using RawTraits = std::allocator_traits<A>;
		using Allocator = typename RawTraits::template rebind_alloc<Node>;
		using Traits = std::allocator_traits<Allocator>;

		Allocator rebound{ alloc };
		Traits::deallocate(rebound, this, 1);
	}

private:
	U value_;
};

template <typename T, typename A, bool IsConst>
class Iterator {
	using Traits = std::allocator_traits<A>;
	using NodeAllocator =
		typename Traits::template rebind_alloc<NodeBase<T, A>>;
	using NodeTraits = std::allocator_traits<NodeAllocator>;
	using NodePointer = std::conditional_t<
		IsConst,
		typename NodeTraits::const_pointer,
		typename NodeTraits::pointer
	>;

public:
	friend PolymorphicList<T, A>;

	using difference_type = typename Traits::difference_type;
	using iterator_category = std::bidirectional_iterator_tag;
	using pointer = std::conditional_t<IsConst, typename Traits::const_pointer,
									   typename Traits::pointer>;
	using reference = std::conditional_t<IsConst, const T&, T&>;
	using value_type = T;

	Iterator() noexcept = default;

	template <std::enable_if_t<
		std::is_same<
			Identity<Iterator<T, A, IsConst>>,
			Iterator<T, A, false>
		>::value, int> = 0
	>
	Iterator(Iterator<T, A, false> other) noexcept
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
		assert(current_->prev);

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

		return pointer{ std::addressof(**this) };
	}

	friend bool operator==(Iterator lhs, Iterator rhs) noexcept {
		return lhs.current_ == rhs.current_;
	}

	friend bool operator!=(Iterator lhs, Iterator rhs) noexcept {
		return !(lhs == rhs);
	}

private:
	Iterator(NodePointer current, bool is_past_end) noexcept
	: current_{ current }, is_past_end_{ is_past_end } { }

	NodePointer current_ = nullptr;
	bool is_past_end_ = true;
};

} // namespace detail
} // namespace plist

#endif
