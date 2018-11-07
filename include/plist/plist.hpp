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
	using iterator = detail::Iterator<T, false>;
	using const_iterator = detail::Iterator<T, true>;
	using reverse_iterator = std::reverse_iterator<iterator>;
	using const_reverse_iterator = std::reverse_iterator<const_iterator>;

	~PolymorphicList() {
		clear();

		const auto deleter = dummy_->get_deleter();
		deleter(alloc_, dummy_);
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
		return iterator{ std::addressof(*head_)  };
	}

	iterator end() noexcept {
		return iterator{ std::addressof(*dummy_) };
	}

	const_iterator begin() const noexcept {
		return cbegin();
	}

	const_iterator end() const noexcept {
		return cend();
	}

	const_iterator cbegin() const noexcept {
		return const_iterator{ std::addressof(*head_) };
	}

	const_iterator cend() const noexcept {
		return iterator{ std::addressof(*dummy_) };
	}

	reverse_iterator rbegin() noexcept {
		return reverse_iterator{ end() };
	}

	const_reverse_iterator rbegin() const noexcept {
		return crbegin();
	}

	const_reverse_iterator crbegin() const noexcept {
		return const_reverse_iterator{ end() };
	}

	reverse_iterator rend() noexcept {
		return reverse_iterator{ begin() };
	}

	const_reverse_iterator rend() const noexcept {
		return crend();
	}

	const_reverse_iterator crend() const noexcept {
		return const_reverse_iterator{ begin() };
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
		for (auto current = head_; current != nullptr && current != dummy_;) {
			const auto next = current->next;
			const auto deleter = current->get_deleter();
			deleter(alloc_, current);
			current = static_cast<NodePointer>(next);
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
		return *emplace<U>(cend(), std::forward<As>(args)...);
	}

	void pop_back() {
		erase(std::prev(cend()));
	}

	template <typename U,
			  std::enable_if_t<std::is_same<T, U>::value || std::is_base_of<T, U>::value, int> = 0>
	void push_front(const U &u) {
		emplace_front<U>(u);
	}

	template <typename U,
			  std::enable_if_t<std::is_same<T, U>::value || std::is_base_of<T, U>::value, int> = 0>
	void push_front(U &&u) {
		emplace_front<U>(std::move(u));
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
	T& emplace_front(As &&...args) {
		return *emplace<U>(cbegin(), std::forward<As>(args)...);
	}

	void pop_front() {
		erase(cbegin());
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
	iterator emplace(const_iterator pos, As &&...args) {
		using PtrTraits = std::pointer_traits<NodePointer>;

		assert(pos.current_);

		const auto node =
			allocate_and_construct_node<U>(std::forward<As>(args)...);

		const auto pos_node =
			PtrTraits::pointer_to(static_cast<Node&>(*pos.current_));

		return iterator{ to_raw_pointer(do_insert(pos_node, node)) };
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
	iterator erase(const_iterator pos) {
		assert(pos.current_ != dummy_);

		const auto pos_node =
			static_cast<gsl::owner<NodePointer>>(pos.current_);
		const auto after_node = do_erase(pos_node);

		if (!after_node) {
			return end();
		}

		return iterator{ after_node };
	}

private:
	using Node = detail::NodeBaseWithAllocator<T, A>;
	using NodeAllocator =
		typename std::allocator_traits<A>::template rebind_alloc<Node>;
	using NodeTraits = std::allocator_traits<NodeAllocator>;
	using NodePointer = typename NodeTraits::pointer;

	gsl::owner<NodePointer> allocate_dummy() {
		using DerivedNode = detail::DummyNode<T, A>;
		using DerivedAllocator =
			typename NodeTraits::template rebind_alloc<DerivedNode>;
		using DerivedTraits = std::allocator_traits<DerivedAllocator>;

		DerivedAllocator alloc{ alloc_ };
		const auto allocated = DerivedTraits::allocate(alloc, 1);
		DerivedTraits::construct(alloc, allocated);

		return allocated;
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

	NodePointer do_insert(NodePointer pos, gsl::owner<NodePointer> node) noexcept {
		assert(pos);
		assert(node);
		assert(!node->next);
		assert(!node->prev);

		node->next = to_raw_pointer(pos);
		node->prev = pos->prev;

		if (pos->prev) {
			pos->prev->next = to_raw_pointer(node);
		}

		pos->prev = to_raw_pointer(node);

		if (pos == dummy_ || tail_ == dummy_) {
			tail_ = node;
			dummy_->prev = tail_;
			tail_->next = dummy_;
		}

		if (pos == head_ || head_ == dummy_) {
			head_ = node;
		}

		++size_;

		return node;
	}

	Node* do_erase(gsl::owner<NodePointer> node) noexcept {
		assert(node);
		assert(node != dummy_);

		--size_;

		const auto after = static_cast<Node*>(node->next);

		if (!node->prev) {
			assert(node == head_);
			head_ = to_alloc_pointer(head_->next);

			if (head_) {
				head_->prev = nullptr;
			}
		} else {
			assert(node != head_);

			node->prev->next = node->next;
		}

		if (node->next == to_raw_pointer(dummy_)) {
			assert(node == tail_);

			dummy_->prev = tail_->prev;

			if (tail_->prev) {
				tail_->prev->next = dummy_;
				tail_ = to_alloc_pointer(tail_->prev);
			} else {
				tail_ = dummy_;
			}
		} else {
			assert(node != tail_);

			node->next->prev = node->prev;
		}

		node->next = nullptr;
		node->prev = nullptr;

		const auto deleter = node->get_deleter();
		deleter(alloc_, node);

		return after;
	}

	static NodePointer to_alloc_pointer(Node *node) {
		if (!node) {
			return NodePointer{ };
		}

		return std::pointer_traits<NodePointer>::to_pointer(*node);
	}

	static Node* to_raw_pointer(NodePointer node) {
		if (!node) {
			return nullptr;
		}

		return std::addressof(*node);
	}

	allocator_type alloc_;
	gsl::owner<NodePointer> dummy_ = allocate_dummy();
	gsl::owner<NodePointer> head_ = dummy_;
	gsl::owner<NodePointer> tail_ = dummy_;
	size_type size_ = 0;
};

} // namespace plist

#endif
