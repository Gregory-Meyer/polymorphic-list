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

#ifndef PLIST_PLIST_HPP
#define PLIST_PLIST_HPP

#include <plist/detail.hpp>

#include <limits>
#include <memory>
#include <type_traits>
#include <utility>

#include <gsl/gsl>

namespace plist {

// A defaults to std::allocator
template <typename T, typename A>
class PolymorphicList {
public:
	using allocator_type = A;
	using const_pointer = typename std::allocator_traits<A>::const_pointer;
	using const_reference = const T&;
	using pointer = typename std::allocator_traits<A>::pointer;
	using reference = T&;
	using size_type = typename std::allocator_traits<A>::size_type;
	using value_type = T;
	using iterator = detail::Iterator<T, A, false>;
	using const_iterator = detail::Iterator<T, A, true>;

	~PolymorphicList() {
		clear();
	}

	reference front() {
		assert(head_);

		return head_->value();
	}

	const_reference front() const {
		assert(head_);

		return head_->value();
	}

	reference back() {
		assert(tail_);

		return tail_->value();
	}

	const_reference back() const {
		assert(tail_);

		return tail_->value();
	}

	iterator begin() noexcept {
		if (empty()) {
			return end();
		}

		return iterator{ head_, false };
	}

	iterator end() noexcept {
		return iterator{ tail_, true };
	}

	const_iterator begin() const noexcept {
		return cbegin();
	}

	const_iterator end() const noexcept {
		return cend();
	}

	const_iterator cbegin() const noexcept {
		if (empty()) {
			return cend();
		}

		return const_iterator{ head_, false };
	}

	const_iterator cend() const noexcept {
		return const_iterator{ tail_, true };
	}

	bool empty() const noexcept {
		return size() == 0;
	}

	size_type size() const noexcept {
		return size_;
	}

	size_type max_size() const noexcept {
		return std::numeric_limits<size_type>::max();
	}

	void clear() noexcept {
		for (auto current = head_; current != nullptr;) {
			const auto next = current->next;
			current->deallocate(alloc_);
			current = next;
		}

		head_ = nullptr;
		tail_ = nullptr;
		size_ = 0;
	}

	template <typename U,
			  std::enable_if_t<std::is_same<T, U>::value || std::is_base_of<T, U>::value, int> = 0>
	void push_back(const U &u) {
		emplace_back<U>(u);
	}

	template <typename U,
			  std::enable_if_t<std::is_same<T, U>::value || std::is_base_of<T, U>::value, int> = 0>
	void push_back(U &&u) {
		emplace_back<U>(std::move(u));
	}

	template <
		typename U,
		typename ...As,
		std::enable_if_t<
			(std::is_same<T, U>::value || std::is_base_of<T, U>::value)
			&& std::is_constructible<U, As&&...>::value,
			int
		> = 0
	>
	T& emplace_back(As &&...args) {
		const auto node =
			allocate_and_construct_node<U>(std::forward<As>(args)...);

		if (empty()) {
			head_ = node;
		} else if (tail_) {
			tail_->next = node;
		}

		node->prev = tail_;
		tail_ = node;
		++size_;

		return node->value();
	}

	void pop_back() {
		assert(tail_);

		const auto new_tail = tail_->prev;

		if (new_tail) {
			new_tail->next = nullptr;
		} else {
			head_ = nullptr;
		}

		tail_->deallocate(alloc_);
		tail_ = new_tail;

		--size_;
	}

private:
	using Node = detail::NodeBase<T, A>;
	using NodeAllocator =
		typename std::allocator_traits<A>::template rebind_alloc<Node>;
	using NodeTraits = std::allocator_traits<NodeAllocator>;
	using NodePointer = typename NodeTraits::pointer;

	template <
		typename U,
		typename ...As,
		std::enable_if_t<
			(std::is_same<T, U>::value || std::is_base_of<T, U>::value)
			&& std::is_constructible<U, As&&...>::value,
			int
		> = 0
	>
	gsl::owner<NodePointer> allocate_and_construct_node(As &&...args) {
		using DerivedNode = detail::Node<U, T, A>;
		using DerivedAllocator =
			typename NodeTraits::template rebind_alloc<DerivedNode>;
		using DerivedTraits = std::allocator_traits<DerivedAllocator>;

		DerivedAllocator alloc{ alloc_ };
		const auto allocated =
			DerivedTraits::allocate(alloc, 1);

		try {
			DerivedTraits::construct(alloc, allocated,
									 std::forward<As>(args)...);
		} catch (...) {
			DerivedTraits::deallocate(alloc, allocated, 1);

			throw;
		}

		return allocated;
	}

	gsl::owner<NodePointer> head_ = nullptr;
	gsl::owner<NodePointer> tail_ = nullptr;
	NodeAllocator alloc_;
	size_type size_ = 0;
};

} // namespace plist

#endif
