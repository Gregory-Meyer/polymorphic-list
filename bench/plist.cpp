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

#include <plist/plist.hpp>

#include <cassert>

#include <algorithm>
#include <chrono>
#include <list>
#include <memory>
#include <random>
#include <vector>

#include <benchmark/benchmark.h>

struct Value {
	virtual ~Value() = default;

	virtual int get() const noexcept = 0;
};

struct Zero : Value {
	virtual ~Zero() = default;

	int get() const noexcept override {
		return 0;
	}
};

struct One : Value {
	virtual ~One() = default;

	int get() const noexcept override {
		return 1;
	}
};

struct Two : Value {
	virtual ~Two() = default;

	int get() const noexcept override {
		return 2;
	}
};

using List = std::list<std::unique_ptr<Value>>;
using Plist = plist::PolymorphicList<Value>;

static void push_value(List &values, int value) {
	switch (value) {
	case 0:
		values.push_back(std::make_unique<Zero>());
		break;
	case 1:
		values.push_back(std::make_unique<One>());
		break;
	case 2:
		values.push_back(std::make_unique<Two>());
		break;
	default:
		__builtin_unreachable();
	}
}

static void push_value(Plist &values, int value) {
	switch (value) {
	case 0:
		values.emplace_back<Zero>();
		break;
	case 1:
		values.emplace_back<One>();
		break;
	case 2:
		values.emplace_back<Two>();
		break;
	default:
		__builtin_unreachable();
	}
}

static int sum(const List &list) {
	return std::accumulate(
		list.cbegin(),
		list.cend(),
		0,
		[](int prev, List::const_reference value) {
			return prev + value->get();
		}
	);
}

static int sum(const Plist &list) {
	return std::accumulate(
		list.cbegin(),
		list.cend(),
		0,
		[](int prev, Plist::const_reference value) {
			return prev + value.get();
		}
	);
}

static int get_front(const List &list) noexcept {
	assert(!list.empty());

	return list.front()->get();
}

static int get_front(const Plist &list) noexcept {
	assert(!list.empty());

	return list.front().get();
}

static int get(List::const_reference elem) noexcept {
	return elem->get();
}

static int get(Plist::const_reference elem) noexcept {
	return elem.get();
}

template <typename T>
T make_random_list(std::mt19937 &gen, typename T::size_type n) {
	using Size = typename T::size_type;

	std::uniform_int_distribution<int> dist{ 0, 3 };
	T values_list;

	for (Size i = 0; i < n; ++i) {
		push_value(values_list, dist(gen));
	}

	return values_list;
}

template <typename T>
static void bm_push(benchmark::State &state) {
	for (auto _ : state) {
		T values_list;

		const auto start = std::chrono::steady_clock::now();

		push_value(values_list, 0);

		const auto end = std::chrono::steady_clock::now();
		const std::chrono::duration<double> elapsed = end - start;
		state.SetIterationTime(elapsed.count());
	}
}
BENCHMARK_TEMPLATE(bm_push, List)->UseManualTime();
BENCHMARK_TEMPLATE(bm_push, Plist)->UseManualTime();

template <typename T>
static void bm_sum(benchmark::State &state) {
	using Size = typename T::size_type;

	const auto gen_ptr = std::make_unique<std::mt19937>();

	const auto values_list =
		make_random_list<T>(*gen_ptr, static_cast<Size>(state.range(0)));

	for (auto _ : state) {
		benchmark::DoNotOptimize(sum(values_list));
	}
}
BENCHMARK_TEMPLATE(bm_sum, List)->Range(1, 1 << 16);
BENCHMARK_TEMPLATE(bm_sum, Plist)->Range(1, 1 << 16);

template <typename T>
static void bm_clear(benchmark::State &state) {
	using Size = typename T::size_type;

	const auto gen_ptr = std::make_unique<std::mt19937>();

	for (auto _ : state) {
		auto values_list =
			make_random_list<T>(*gen_ptr, static_cast<Size>(state.range(0)));

		const auto start = std::chrono::steady_clock::now();

		values_list.clear();

		const auto end = std::chrono::steady_clock::now();
		const std::chrono::duration<double> elapsed = end - start;
		state.SetIterationTime(elapsed.count());
	}
}
BENCHMARK_TEMPLATE(bm_clear, List)->Range(1, 1 << 16)->UseManualTime();
BENCHMARK_TEMPLATE(bm_clear, Plist)->Range(1, 1 << 16)->UseManualTime();

template <typename T>
static void bm_front(benchmark::State &state) {
	const auto gen_ptr = std::make_unique<std::mt19937>();
	const auto values_list = make_random_list<T>(*gen_ptr, 4);

	for (auto _ : state) {
		for (int i = 0; i < 32; ++i) {
			benchmark::DoNotOptimize(get_front(values_list));
		}
	}
}
BENCHMARK_TEMPLATE(bm_front, List);
BENCHMARK_TEMPLATE(bm_front, Plist);

template <typename T>
static void bm_iterate(benchmark::State &state) {
	using Size = typename T::size_type;

	const auto gen_ptr = std::make_unique<std::mt19937>();
	const auto values_list =
			make_random_list<T>(*gen_ptr, static_cast<Size>(state.range(0)));

	for (auto _ : state) {
		benchmark::DoNotOptimize(
			std::distance(values_list.cbegin(), values_list.cend())
		);
	}
}
BENCHMARK_TEMPLATE(bm_iterate, List)->Range(1, 1 << 16);
BENCHMARK_TEMPLATE(bm_iterate, Plist)->Range(1, 1 << 16);

template <typename T>
static void bm_read_iterate(benchmark::State &state) {
	using Size = typename T::size_type;

	const auto gen_ptr = std::make_unique<std::mt19937>();
	const auto values_list =
			make_random_list<T>(*gen_ptr, static_cast<Size>(state.range(0)));

	for (auto _ : state) {
		std::vector<int> v;
		v.reserve(values_list.size());

		const auto start = std::chrono::steady_clock::now();

		for (const auto &elem : values_list) {
			v.push_back(get(elem));
		}

		const auto end = std::chrono::steady_clock::now();
		const std::chrono::duration<double> elapsed = end - start;
		state.SetIterationTime(elapsed.count());
	}
}
BENCHMARK_TEMPLATE(bm_read_iterate, List)->Range(1, 1 << 16)->UseManualTime();
BENCHMARK_TEMPLATE(bm_read_iterate, Plist)->Range(1, 1 << 16)->UseManualTime();
