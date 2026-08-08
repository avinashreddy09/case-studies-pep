/*
 * AI-Based Stock Market Event Correlation Platform
 * ===================================================
 * Case Study #42 — Full C++ implementation of all components:
 *
 *   1. Queue          -> incoming news processed in order        O(1)
 *   2. Quick Sort      -> ranking events by market impact          avg O(n log n), worst O(n^2)
 *   3. Binary Search    -> instant event record lookup               O(log n)
 *   4. BFS               -> impact spread through company/sector graph  O(V + E)
 *
 * Compile:  g++ -std=c++17 -O2 -o stockcorr stock_event_correlation_platform.cpp
 * Run:      ./stockcorr
 */

#include <iostream>
#include <vector>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <optional>
#include <sstream>
#include <random>

// ---------------------------------------------------------------------------
// 1. QUEUE — Incoming news processed in arrival order
// ---------------------------------------------------------------------------
struct NewsItem {
    std::string headline;
    std::string company;  // company/sector this news is tied to
};

class NewsQueue {
public:
    void enqueue(const NewsItem& item) { queue_.push(item); }

    std::optional<NewsItem> dequeue() {
        if (queue_.empty()) return std::nullopt;
        NewsItem item = queue_.front();
        queue_.pop();
        return item;
    }

    // Drain the queue, processing each item in the order it arrived —
    // news must be handled chronologically so downstream ranking and
    // correlation reflect the real sequence of market-moving events.
    std::vector<std::string> processAll() {
        std::vector<std::string> log;
        while (auto item = dequeue()) {
            log.push_back("[PROCESSED] " + item->company + ": " + item->headline);
        }
        return log;
    }

    size_t size() const { return queue_.size(); }

private:
    std::queue<NewsItem> queue_;
};

// ---------------------------------------------------------------------------
// 2. QUICK SORT — Rank events by market impact
// ---------------------------------------------------------------------------
struct MarketEvent {
    std::string id;         // sorted key for binary search, e.g. "EVT-0001"
    std::string headline;
    std::string company;
    double impactScore;     // e.g. predicted % price move, positive or negative magnitude
};

// Quick sort ranked by |impactScore| descending (biggest movers first).
// Average O(n log n); worst case O(n^2) on already-sorted/adversarial
// input, which random pivot selection guards against in practice.
void quickSortByImpact(std::vector<MarketEvent>& events, int low, int high) {
    if (low >= high) return;

    static std::mt19937 rng(std::random_device{}());
    std::uniform_int_distribution<int> dist(low, high);
    std::swap(events[dist(rng)], events[high]);  // random pivot -> avoid worst case

    double pivot = std::abs(events[high].impactScore);
    int i = low - 1;
    for (int j = low; j < high; ++j) {
        if (std::abs(events[j].impactScore) > pivot) {  // descending order
            ++i;
            std::swap(events[i], events[j]);
        }
    }
    std::swap(events[i + 1], events[high]);
    int pivotIndex = i + 1;

    quickSortByImpact(events, low, pivotIndex - 1);
    quickSortByImpact(events, pivotIndex + 1, high);
}

void quickSortByImpact(std::vector<MarketEvent>& events) {
    if (!events.empty()) quickSortByImpact(events, 0, static_cast<int>(events.size()) - 1);
}

// ---------------------------------------------------------------------------
// 3. BINARY SEARCH — Instant event record lookup by ID
// ---------------------------------------------------------------------------
class EventRegistry {
public:
    // Keeps events_ sorted by id so lookups are O(log n). Insert is O(n),
    // the right trade-off when the registry is queried far more often
    // than it's written to (dashboards, alerts, analyst tools).
    void addEvent(const MarketEvent& event) {
        auto it = std::lower_bound(
            events_.begin(), events_.end(), event,
            [](const MarketEvent& a, const MarketEvent& b) { return a.id < b.id; });
        events_.insert(it, event);
    }

    std::optional<MarketEvent> lookup(const std::string& id) const {
        int lo = 0, hi = static_cast<int>(events_.size()) - 1;
        while (lo <= hi) {
            int mid = lo + (hi - lo) / 2;
            if (events_[mid].id == id) return events_[mid];
            if (events_[mid].id < id) lo = mid + 1;
            else hi = mid - 1;
        }
        return std::nullopt;
    }

private:
    std::vector<MarketEvent> events_;
};

// ---------------------------------------------------------------------------
// 4. GRAPH — BFS: how an event's impact spreads through connected companies
// ---------------------------------------------------------------------------
class CorrelationGraph {
public:
    void addConnection(const std::string& a, const std::string& b) {
        adj_[a].push_back(b);
        adj_[b].push_back(a);
    }

    // Breadth-first search: spreads outward one "ring" of connection at a
    // time (like ripples from a stone dropped in water), so companies
    // directly linked to the disrupted one are flagged before companies
    // two or three hops removed. Naturally gives each affected company
    // its distance (hops) from the source event. O(V + E).
    std::vector<std::pair<std::string, int>> bfsImpactSpread(
        const std::string& source, int maxHops = -1) const {
        std::vector<std::pair<std::string, int>> affected;
        std::unordered_set<std::string> visited{source};
        std::queue<std::pair<std::string, int>> q;
        q.push({source, 0});

        while (!q.empty()) {
            auto [node, dist] = q.front();
            q.pop();
            if (node != source) affected.push_back({node, dist});

            if (maxHops != -1 && dist >= maxHops) continue;

            auto it = adj_.find(node);
            if (it == adj_.end()) continue;
            for (const auto& neighbor : it->second) {
                if (!visited.count(neighbor)) {
                    visited.insert(neighbor);
                    q.push({neighbor, dist + 1});
                }
            }
        }
        return affected;
    }

private:
    std::unordered_map<std::string, std::vector<std::string>> adj_;
};

// ---------------------------------------------------------------------------
// DEMO
// ---------------------------------------------------------------------------
int main() {
    std::cout << std::string(70, '=') << "\n";
    std::cout << "1. QUEUE - Incoming news processed in order\n";
    std::cout << std::string(70, '=') << "\n";
    NewsQueue news;
    news.enqueue({"Q2 earnings beat expectations", "TechCorp"});
    news.enqueue({"Regulatory probe announced", "PharmaCo"});
    news.enqueue({"Supply chain disruption at key plant", "ChipMakerX"});
    for (const auto& line : news.processAll()) {
        std::cout << "  " << line << "\n";
    }

    std::cout << "\n" << std::string(70, '=') << "\n";
    std::cout << "2. QUICK SORT - Rank events by market impact\n";
    std::cout << std::string(70, '=') << "\n";
    std::vector<MarketEvent> events = {
        {"EVT-0004", "Supply chain disruption at key plant", "ChipMakerX", -8.4},
        {"EVT-0001", "Q2 earnings beat expectations", "TechCorp", 5.1},
        {"EVT-0003", "Regulatory probe announced", "PharmaCo", -3.7},
        {"EVT-0002", "CEO resigns unexpectedly", "RetailCo", -6.9},
        {"EVT-0005", "New product line announced", "TechCorp", 2.2},
    };
    quickSortByImpact(events);
    for (size_t i = 0; i < events.size(); ++i) {
        std::cout << "  #" << (i + 1) << ": " << events[i].company
                   << " — \"" << events[i].headline << "\" ("
                   << events[i].impactScore << "%)\n";
    }

    std::cout << "\n" << std::string(70, '=') << "\n";
    std::cout << "3. BINARY SEARCH - Event record lookup\n";
    std::cout << std::string(70, '=') << "\n";
    EventRegistry registry;
    for (const auto& e : events) registry.addEvent(e);

    for (const auto& query : {std::string("EVT-0004"), std::string("EVT-9999")}) {
        auto result = registry.lookup(query);
        if (result) {
            std::cout << "  Found " << query << ": " << result->company
                       << " (" << result->impactScore << "%)\n";
        } else {
            std::cout << "  " << query << ": not found\n";
        }
    }

    std::cout << "\n" << std::string(70, '=') << "\n";
    std::cout << "4. GRAPH - BFS impact spread through connected companies\n";
    std::cout << std::string(70, '=') << "\n";
    CorrelationGraph graph;
    // ChipMakerX supplies/connects to several companies across sectors
    graph.addConnection("ChipMakerX", "TechCorp");
    graph.addConnection("ChipMakerX", "AutoMakerY");
    graph.addConnection("TechCorp", "SoftwareCo");
    graph.addConnection("AutoMakerY", "PartsSupplierZ");
    graph.addConnection("SoftwareCo", "CloudProviderQ");
    graph.addConnection("ChipMakerX", "ElectronicsRetail");

    std::cout << "  Scenario: Supply chain disruption reported at ChipMakerX.\n";
    auto affected = graph.bfsImpactSpread("ChipMakerX");
    for (const auto& [company, hops] : affected) {
        std::cout << "  " << company << " — " << hops << " hop(s) from source\n";
    }

    return 0;
}
