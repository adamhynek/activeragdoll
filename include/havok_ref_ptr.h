#pragma once

/*
This is a refcounted ptr class to mimic havok's, using function offsets for add/remove reference instead of havok member functions
*/

typedef void(*_hkReferencedObject_addReference)(hkReferencedObject *_this);
extern RelocAddr<_hkReferencedObject_addReference> hkReferencedObject_addReference;

typedef void(*_hkReferencedObject_removeReference)(hkReferencedObject *_this);
extern RelocAddr<_hkReferencedObject_removeReference> hkReferencedObject_removeReference;

namespace RE {
    template <class TYPE>
    class hkRefPtr
    {
    public:
        typedef hkRefPtr<TYPE> ThisType;

        __forceinline hkRefPtr() : m_pntr(nullptr)
        {
        }

        // Copy constructor.
        __forceinline hkRefPtr(const hkRefPtr &rp)
        {
            if (rp.m_pntr)
            {
                hkReferencedObject_addReference(rp.m_pntr);
            }
            m_pntr = rp.m_pntr;
        }

        // Constructor from pointer.
        // Increase reference count for object 'e' and set the pointer to it.
        __forceinline hkRefPtr(TYPE *e)
        {
            if (e)
            {
                hkReferencedObject_addReference(e);
            }
            m_pntr = e;
        }

        // Destructor.
        // Decrease reference count for stored object.
        __forceinline ~hkRefPtr()
        {
            if (m_pntr)
            {
                hkReferencedObject_removeReference(m_pntr);
            }
            m_pntr = nullptr;
        }

        // Assignment operator.
        // Increase reference count for object in 'rp',
        // decrease reference count for stored object and set pointer to object from 'rp'.
        __forceinline void operator=(const hkRefPtr &rp)
        {
            if (rp.m_pntr)
            {
                hkReferencedObject_addReference(rp.m_pntr); // add reference first to allow self-assignment
            }
            if (m_pntr)
            {
                hkReferencedObject_removeReference(m_pntr);
            }
            m_pntr = rp.m_pntr;
        }

        // Assignment operator.
        // Increase reference count for object 'e',
        // decrease reference count for stored object and set pointer to 'e'.
        __forceinline void operator=(TYPE *e)
        {
            if (e)
            {
                hkReferencedObject_addReference(e); // add reference first to allow self-assignment
            }
            if (m_pntr)
            {
                hkReferencedObject_removeReference(m_pntr);
            }
            m_pntr = e;
        }

        // Return pointer to stored object.
        __forceinline TYPE *val() const
        {
            return m_pntr;
        }

        // Pointer to stored object.
        __forceinline TYPE *operator->() const
        {
            return m_pntr;
        }

        // Replace stored pointer with 'e' without incrementing reference count for 'e'.
        // Reference count for previously stored object is decreased.
        __forceinline void setAndDontIncrementRefCount(TYPE *e)
        {
            if (m_pntr && m_pntr != e)
            {
                hkReferencedObject_removeReference(m_pntr);
            }
            m_pntr = e;
        }

        // Return pointer to stored object.
        __forceinline operator TYPE *() const
        {
            return val();
        }

    private:

        TYPE *m_pntr;
    };
}
