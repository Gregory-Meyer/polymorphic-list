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

template <typename T>
struct Node;

template <typename T>
struct TypeTag { };

using Storage = std::aligned_storage_t<sizeof(float) * 8, alignof(std::max_align_t)>;

template <typename U>
static constexpr bool can_store() noexcept {
	return sizeof(U) <= sizeof(Storage) && alignof(U) <= alignof(Storage);
}

template <typename T>
class SmallBox {
public:
	template <typename U, typename ...Ts, std::enable_if_t<(std::is_same<T, U>::value || std::is_base_of<T, U>::value) && std::is_constructible<U, Ts&&...>::value && can_store<U>(), int> = 0>
	explicit SmallBox(TypeTag<U>, Ts &&...ts) noexcept(std::is_nothrow_constructible<U, Ts&&...>::value) {
		new (static_cast<void*>(&storage_)) U(std::forward<Ts>(ts)...);
		is_local_ = true;
	}

	template <typename U, typename ...Ts, std::enable_if_t<(std::is_same<T, U>::value || std::is_base_of<T, U>::value) && std::is_constructible<U, Ts&&...>::value && !can_store<U>(), int> = 0>
	explicit SmallBox(TypeTag<U>, Ts &&...ts) {
		new (static_cast<void*>(&box_)) Box(std::make_unique<U>(std::forward<Ts>(ts)...));
		is_local_ = false;
	}

	~SmallBox() {
		if (is_local_) {
			get().T::~T();
		} else {
			box_.Box::~Box();
		}
	}

	T& get() & noexcept {
		if (is_local_) {
			return reinterpret_cast<T&>(storage_);
		} else {
			return *box_;
		}
	}

	const T& get() const & noexcept {
		if (is_local_) {
			return reinterpret_cast<const T&>(storage_);
		} else {
			return *box_;
		}
	}

	T&& get() && noexcept {
		if (is_local_) {
			return std::move(reinterpret_cast<T&>(storage_));
		} else {
			return std::move(*box_);
		}
	}

	const T&& get() const && noexcept {
		if (is_local_) {
			return std::move(reinterpret_cast<const T&>(storage_));
		} else {
			return std::move(*box_);
		}
	}

	T& operator*() & noexcept {
		return get();
	}

	const T& operator*() const & noexcept {
		return get();
	}

	T&& operator*() && noexcept {
		return get();
	}

	const T&& operator*() const && noexcept {
		return get();
	}

private:
	template <typename U>
	friend struct Node;

	using Box = std::unique_ptr<T>;

	SmallBox() noexcept {
		new (std::addressof(box_)) Box{ };
		is_local_ = false;
	}

	union {
		Storage storage_;
		Box box_;
	};

	bool is_local_ = true;
};

template <typename T>
struct Node {
	template <typename ...Ts, std::enable_if_t<std::is_constructible<SmallBox<T>, Ts&&...>::value, int> = 0>
	explicit Node(Ts &&...ts) noexcept(std::is_nothrow_constructible<SmallBox<T>, Ts&&...>::value)
	: value{ std::forward<Ts>(ts)... } { }

	SmallBox<T> value;
	Node *next = nullptr;
	Node *prev = nullptr;

private:
	template <typename U, typename A>
	friend class ::plist::PolymorphicList;

	Node() noexcept = default;
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

		return *current_->value;
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
	explicit Iterator(Node<T> *current) noexcept : current_{ current } { }

	Node<T> *current_ = nullptr;
};

} // namespace detail
} // namespace plist

#endif
