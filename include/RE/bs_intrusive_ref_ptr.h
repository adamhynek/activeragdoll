#pragma once

#include <Windows.h>

#include <cstdint>
#include <type_traits>
#include <utility>

// SKSE VR's legacy BSTSmartPointer is layout-only and does not decrement
// BSIntrusiveRefCounted references. Use this owner for engine APIs that AddRef
// an output parameter. It deliberately stays pointer-sized so it is ABI
// compatible with the engine's output wrapper.
template <class T, std::size_t RefCountOffset>
class BSIntrusiveRefPtr
{
public:
    T *ptr{ nullptr };

    BSIntrusiveRefPtr() noexcept = default;
    BSIntrusiveRefPtr(const BSIntrusiveRefPtr &) = delete;
    BSIntrusiveRefPtr &operator=(const BSIntrusiveRefPtr &) = delete;

    BSIntrusiveRefPtr(BSIntrusiveRefPtr &&other) noexcept :
        ptr(std::exchange(other.ptr, nullptr))
    {}

    BSIntrusiveRefPtr &operator=(BSIntrusiveRefPtr &&other) noexcept
    {
        if (this != &other) {
            Reset();
            ptr = std::exchange(other.ptr, nullptr);
        }
        return *this;
    }

    ~BSIntrusiveRefPtr()
    {
        Reset();
    }

    void Reset() noexcept
    {
        T *object = std::exchange(ptr, nullptr);
        auto *refCount = object ? reinterpret_cast<volatile LONG *>(
            reinterpret_cast<std::uintptr_t>(object) + RefCountOffset) : nullptr;
        if (object && InterlockedDecrement(refCount) == 0) {
            using Destruct = void(*)(T *, std::uint32_t);
            auto *vtable = *reinterpret_cast<std::uintptr_t **>(object);
            reinterpret_cast<Destruct>(vtable[0])(object, 1);
        }
    }
};
