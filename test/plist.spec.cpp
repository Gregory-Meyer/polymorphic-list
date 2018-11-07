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

#include <algorithm>
#include <functional>
#include <iterator>
#include <memory>
#include <vector>

#include <catch2/catch.hpp>

struct Base {
	virtual ~Base() = default;

	virtual std::unique_ptr<Base> clone() const {
		return std::make_unique<Base>();
	}

	virtual int foo() const noexcept {
		return 0;
	}
};

struct Derived : Base {
	virtual ~Derived() = default;

	std::unique_ptr<Base> clone() const override {
		return std::make_unique<Derived>();
	}

	int foo() const noexcept override {
		return 1;
	}
};

TEST_CASE("PolymorphicList<int>") {
	plist::PolymorphicList<int> list;
	REQUIRE(list.empty());
	REQUIRE(list.size() == 0);

	list.push_back(15);
	REQUIRE(list.front() == 15);
	REQUIRE(list.back() == 15);
	REQUIRE(list.size() == 1);

	list.emplace_back<int>(20);
	REQUIRE(list.front() == 15);
	REQUIRE(list.back() == 20);
	REQUIRE(list.size() == 2);

	list.push_back(25);
	REQUIRE(list.front() == 15);
	REQUIRE(list.back() == 25);
	REQUIRE(list.size() == 3);

	REQUIRE(std::vector<int>{ list.cbegin(), list.cend() }
			== std::vector<int>{ 15, 20, 25 });
	REQUIRE(std::vector<int>{ list.crbegin(), list.crend() }
			== std::vector<int>{ 25, 20, 15 });

	list.clear();
	REQUIRE(list.empty());
}

TEST_CASE("PolymorphicList<Base>") {
	plist::PolymorphicList<Base> list;

	list.emplace_back<Derived>();
	REQUIRE(list.front().foo() == 1);
	REQUIRE(list.back().foo() == 1);
	REQUIRE(list.size() == 1);

	list.emplace_back<Base>();
	REQUIRE(list.front().foo() == 1);
	REQUIRE(list.back().foo() == 0);
	REQUIRE(list.size() == 2);

	std::vector<std::unique_ptr<Base>> vec;
	vec.reserve(list.size());
	std::transform(list.cbegin(), list.cend(), std::back_inserter(vec),
				   std::mem_fn(&Base::clone));

	REQUIRE(vec.size() == 2);
	REQUIRE(vec[0]->foo() == 1);
	REQUIRE(vec[1]->foo() == 0);

	list.clear();
	REQUIRE(list.empty());
}
