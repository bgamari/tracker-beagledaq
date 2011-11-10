/* tracker - Back-focal plane droplet tracker
 *
 * Copyright © 2010 Ben Gamari
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see http://www.gnu.org/licenses/ .
 *
 * Author: Ben Gamari <bgamari@physics.umass.edu>
 */

#pragma once

#include <vector>
#include <assert.h>

template <typename T>
class ring_buffer {
private:
        std::vector<T> v;
        unsigned int head;
        bool full;
public:
        ring_buffer(unsigned int capacity)
        {
                resize(capacity);
        }

        void add(T a)
        {
                v[head] = a;
                head++;
                if (head == capacity()) {
                        head = 0;
                        full = true;
                }
        }

        // i=0 is last item added
        T& operator[](unsigned int i)
        {
                assert(i < capacity());
                int off = head - i;
                assert(off >= 0 || full);
                if (off < 0)
                        return v[capacity() + off - 1];
                else
                        return v[off];
        }

        void clear()
        {
                head = 0;
                full = false;
        }

        void resize(unsigned int capacity)
        {
                assert(capacity >= 1);
                head = 0;
                full = false;
                v.reserve(capacity);
        }

        unsigned int capacity()
        {
                return v.capacity();
        }

        unsigned int size()
        {
                return full ? capacity() : head;
        }

        T& front()
        {
                return (*this)[0];
        }

        T& back()
        {
                return (*this)[size()-1];
        }
};

