#ifndef PAIRHASH_HPP
#define PAIRHASH_HPP

#include <functional>
#include <utility>

struct pairHash {
    template <class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2>& pair) const {
        return std::hash<T1>()(pair.first) ^ (std::hash<T2>()(pair.second) << 1);
    }
};

#endif // PAIRHASH_HPP