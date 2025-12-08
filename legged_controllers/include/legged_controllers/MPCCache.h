#ifndef MPC_CACHE_HPP_
#define MPC_CACHE_HPP_

#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

struct MPCCacheEntry {
    std::vector<double> feat;
    std::vector<double> solution;
    double trust_radius = 0.1;
    uint64_t last_use = 0;
};

class MPCCache {
   public:
    // MPCCache() : MPCCache(1024, 0.1) {}
    explicit MPCCache(size_t capacity = 4096, double default_delta = 0.1)
        : capacity_(capacity), default_delta_(default_delta) {}
    void setBinWidths(const std::vector<double>& h) {
        std::lock_guard<std::mutex> lg(m_);
        h_ = h;
    }

    const std::vector<double>& getBinWidths() const { return h_; }

    std::string quantizeKey(const std::vector<double>& feat) const {
        std::string s;
        s.reserve(feat.size() * 4);
        for (size_t i = 0; i < feat.size(); ++i) {
            double denom = (h_.size() == feat.size() ? h_[i] : 0.05);
            long long qi = llround(feat[i] / denom);
            s += std::to_string(qi);
            s.push_back(',');
        }
        return s;
    }

    void insert(MPCCacheEntry&& entry) {
        std::lock_guard<std::mutex> lg(m_);
        const std::string key = this->quantizeKey(entry.feat);
        entry.last_use = ++tick_;
        // std::cout << "Original size before push: " << map_.size() << "\n";
        map_[key] = std::move(entry);
        // std::cout << "New entry made. new size: " << map_.size() << "\n";
        if (map_.size() > capacity_) evictLRU();
    }

    size_t size() const {
        std::lock_guard<std::mutex> lg(m_);
        return map_.size();
    }

    MPCCacheEntry* queryNearest(const std::vector<double>& feat, double delta) {
        std::lock_guard<std::mutex> lg(m_);
        const std::string key = this->quantizeKey(feat);
        auto it = map_.find(key);
        if (it == map_.end()) return nullptr;
        MPCCacheEntry& e = it->second;
        double d = euclidean(feat, e.feat);
        if (d <= delta && d <= e.trust_radius + 1e-12) {
            e.last_use = ++tick_;
            return &e;
        }
        return nullptr;
    }

    void clear() {
        std::lock_guard<std::mutex> lg(m_);
        map_.clear();
    }

    void save(std::ostream& os) const {
        std::lock_guard<std::mutex> lg(m_);
        // Write capacity_
        os.write(reinterpret_cast<const char*>(&capacity_), sizeof(size_t));
        // Write default_delta_
        os.write(reinterpret_cast<const char*>(&default_delta_),
                 sizeof(double));
        // Write tick_
        os.write(reinterpret_cast<const char*>(&tick_), sizeof(uint64_t));
        // Write h_
        size_t h_size = h_.size();
        os.write(reinterpret_cast<const char*>(&h_size), sizeof(size_t));
        for (double d : h_) {
            os.write(reinterpret_cast<const char*>(&d), sizeof(double));
        }
        // Write map_.size()
        size_t map_size = map_.size();
        os.write(reinterpret_cast<const char*>(&map_size), sizeof(size_t));
        // Write each entry
        for (const auto& p : map_) {
            const MPCCacheEntry& e = p.second;
            size_t feat_size = e.feat.size();
            os.write(reinterpret_cast<const char*>(&feat_size), sizeof(size_t));
            for (double d : e.feat) {
                os.write(reinterpret_cast<const char*>(&d), sizeof(double));
            }
            size_t sol_size = e.solution.size();
            os.write(reinterpret_cast<const char*>(&sol_size), sizeof(size_t));
            for (double d : e.solution) {
                os.write(reinterpret_cast<const char*>(&d), sizeof(double));
            }
            os.write(reinterpret_cast<const char*>(&e.trust_radius),
                     sizeof(double));
            os.write(reinterpret_cast<const char*>(&e.last_use),
                     sizeof(uint64_t));
        }
    }

    void load(std::istream& is) {
        std::lock_guard<std::mutex> lg(m_);
        map_.clear();
        // Read capacity_
        is.read(reinterpret_cast<char*>(&capacity_), sizeof(size_t));
        // Read default_delta_
        is.read(reinterpret_cast<char*>(&default_delta_), sizeof(double));
        // Read tick_
        is.read(reinterpret_cast<char*>(&tick_), sizeof(uint64_t));
        // Read h_
        size_t h_size;
        is.read(reinterpret_cast<char*>(&h_size), sizeof(size_t));
        h_.resize(h_size);
        for (size_t i = 0; i < h_size; ++i) {
            is.read(reinterpret_cast<char*>(&h_[i]), sizeof(double));
        }
        // Read map_size
        size_t map_size;
        is.read(reinterpret_cast<char*>(&map_size), sizeof(size_t));
        // Read each entry
        for (size_t j = 0; j < map_size; ++j) {
            MPCCacheEntry e;
            size_t feat_size;
            is.read(reinterpret_cast<char*>(&feat_size), sizeof(size_t));
            e.feat.resize(feat_size);
            for (size_t i = 0; i < feat_size; ++i) {
                is.read(reinterpret_cast<char*>(&e.feat[i]), sizeof(double));
            }
            size_t sol_size;
            is.read(reinterpret_cast<char*>(&sol_size), sizeof(size_t));
            e.solution.resize(sol_size);
            for (size_t i = 0; i < sol_size; ++i) {
                is.read(reinterpret_cast<char*>(&e.solution[i]),
                        sizeof(double));
            }
            is.read(reinterpret_cast<char*>(&e.trust_radius), sizeof(double));
            is.read(reinterpret_cast<char*>(&e.last_use), sizeof(uint64_t));
            // Compute key and insert directly (preserve last_use)
            std::string key = quantizeKey(e.feat);
            map_[key] = std::move(e);
        }
        // Safety: ensure capacity invariant even if file was manually edited or
        // saved with bug
        while (map_.size() > capacity_) {
            evictLRU();
        }
    }

   private:
    static double euclidean(const std::vector<double>& a,
                            const std::vector<double>& b) {
        if (a.size() != b.size())
            return std::numeric_limits<double>::infinity();
        double s = 0.0;
        for (size_t i = 0; i < a.size(); ++i) {
            double d = a[i] - b[i];
            s += d * d;
        }
        return std::sqrt(s);
    }
    void evictLRU() {
        uint64_t oldest = UINT64_MAX;
        std::string oldest_key;
        for (auto& p : map_) {
            if (p.second.last_use < oldest) {
                oldest = p.second.last_use;
                oldest_key = p.first;
            }
        }
        if (!oldest_key.empty()) {
            std::cout << "Oldest key evicted from cache\n";
            map_.erase(oldest_key);
        }
    }
    std::unordered_map<std::string, MPCCacheEntry> map_;
    std::vector<double> h_;
    size_t capacity_;
    double default_delta_;
    mutable std::mutex m_;
    uint64_t tick_ = 0;
};

// Free functions to save/load a vector<unique_ptr<MPCCache>> to/from a single
// file
inline bool save_caches(const std::vector<std::unique_ptr<MPCCache>>& caches,
                        const std::string& filename) {
    std::ofstream os(filename, std::ios::binary);
    if (!os) return false;  // Error handling omitted for simplicity
    size_t sz = caches.size();
    os.write(reinterpret_cast<const char*>(&sz), sizeof(size_t));
    for (const auto& cache : caches) {
        cache->save(os);
    }

    return true;
}

inline bool load_caches(std::vector<std::unique_ptr<MPCCache>>& caches,
                        const std::string& filename) {
    std::ifstream is(filename, std::ios::binary);
    if (!is) return false;  // Error handling omitted for simplicity
    size_t sz;
    is.read(reinterpret_cast<char*>(&sz), sizeof(size_t));
    caches.clear();
    caches.reserve(sz);
    for (size_t i = 0; i < sz; ++i) {
        auto cache =
            std::make_unique<MPCCache>();  // Default construct with default
                                           // capacity/default_delta
        cache->load(is);
        caches.push_back(std::move(cache));
    }

    return true;
}


#endif