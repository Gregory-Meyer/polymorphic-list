### Examples

This library is meant to replace boxed lists such as the following:

```c++
#include <fstream>
#include <iostream>
#include <list>
#include <memory>
#include <sstream>

std::list<std::unique_ptr<std::ostream>> ostreams;

ostreams.push_back(std::make_unique<std::ofstream>("foo.txt"));
ostreams.push_back(std::make_unique<std::ostringstream>());
```

With the following:

```c++
#include <plist/plist.hpp>

#include <iostream>
#include <fstream>
#include <sstream>

plist::PolymorphicList<std::ostream> ostreams;

ostreams.emplace_back<std::ofstream>("foo.txt");
ostreams.emplace_back<std::ostringstream>();
```

With a custom allocator:

```c++
#include <plist/plist.hpp>

#include <cstdlib>
#include <cstddef>

#include <new>

template <typename T>
struct Mallocator {
	using value_type = T;

	Mallocator() noexcept = default;

	template <typename U, std::enable_if_t<!std::is_same<T, U>::value, int> = 0>
	Mallocator(const Mallocator<U>&) noexcept { }

	template <typename U, std::enable_if_t<!std::is_same<T, U>::value, int> = 0>
	Mallocator& operator=(const Mallocator<U>&) noexcept {
		return *this;
	}

	T* allocate(std::size_t n) {
		const auto allocated = reinterpret_cast<T*>(malloc(sizeof(T) * n));

		if (!allocated) {
			throw std::bad_alloc{ };
		}

		return allocated;
	}

	void deallocate(T *ptr, std::size_t) noexcept {
		free(ptr);
	}

	friend bool operator==(const Mallocator&, const Mallocator&) noexcept {
		return true;
	}

	friend bool operator!=(const Mallocator&, const Mallocator&) noexcept {
		return false;
	}
};

plist::PolymorphicList<std::ostream, Mallocator<std::ostream>> ostreams;

ostreams.emplace_back<std::ofstream>("foo.txt");
ostreams.emplace_back<std::ostringstream>();
```

Unlike a linked-list of boxed elements, a PolymorphicList has no indirection
between a node and the element it stores.

### Implementation Details

`PolymorphicList`s are doubly-linked lists where the nodes are polymorphic. To
avoid virtual function call overhead, nodes are immobile (non-copyable and
non-movable) and internally cache the address of their owned element. There is
a further abstract base class that caches a deallocation function to again
avoid virtual function call overhead -- concrete node objects set the value of
the element pointer and deleter function upon construction for later retrieval.
As a result, creating and destroying new elements in a PolymorphicList results
in a single allocation.

`PolymorphicList::[const_]iterator`s are SCARY and non-allocator aware;
`iterator`s can be compared with `iterator`s from containers with different
allocator types.
