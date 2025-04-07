#pragma once

#include <cmath>
#include <deque>
#include <iostream>
#include <numeric>
#include <vector>

template <typename T, typename Total, size_t N>
class MovingAverage {
   public:
    void new_entry(T sample) {
        if (std::isnan(sample)) {
            return;
        }
        if (fifo.size() < N) {
            fifo.push_back(sample);
        } else {
            fifo.pop_front();
            fifo.push_back(sample);
        }
    }

    void clear() { fifo.clear(); }

    T mean() {
        T total = 0;

        // definition: empty mean should be zero
        if (fifo.empty()) return total;

        for (auto& v : fifo) {
            total += v;
        }
        return total / fifo.size();
    }

   private:
    std::deque<T> fifo;
};
