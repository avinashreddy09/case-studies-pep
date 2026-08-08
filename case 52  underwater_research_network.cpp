/*
 * Autonomous Underwater Research Network
 * =========================================
 * Case Study #52 — Full C++ implementation of all components:
 *
 *   1. Queue           -> incoming sensor data processed in order   O(1)
 *   2. Quick Sort       -> ranking exploration targets by importance  avg O(n log n), worst O(n^2)
 *   3. Binary Search     -> instant lookup of past research records    O(log n)
 *   4. Dijkstra's Algo    -> cheapest travel route between stations      O(E log V)
 *   5. BFS                 -> verify all vehicles stay connected           O(V + E)
 *
 * Compile:  g++ -std=c++17 -O2 -o auv underwater_research_network.cpp
 * Run:      ./auv
 */

#include <iostream>
#include <vector>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <optional>
#include <limits>
#include <random>

// ---------------------------------------------------------------------------
// 1. QUEUE — Incoming sensor data processed in order
// ---------------------------------------------------------------------------
struct SensorReading {
    std::string vehicleId;
    std::string reading;  // e.g. "coral formation detected", "temperature spike"
};

class SensorQueue {
public:
    void enqueue(const SensorReading& r) { queue_.push(r); }

    std::optional<SensorReading> dequeue() {
        if (queue_.empty()) return std::nullopt;
        SensorReading r = queue_.front();
        queue_.pop();
        return r;
    }

    // Drain in FIFO order — sensor data must be processed in the sequence
    // it was collected so downstream priority ranking reflects reality.
    std::vector<std::string> processAll() {
        std::vector<std::string> log;
        while (auto r = dequeue()) {
            log.push_back("[DATA] " + r->vehicleId + ": " + r->reading);
        }
        return log;
    }

    size_t size() const { return queue_.size(); }

private:
    std::queue<SensorReading> queue_;
};

// ---------------------------------------------------------------------------
// 2. QUICK SORT — Rank exploration targets by scientific importance
// ---------------------------------------------------------------------------
struct ExplorationTarget {
    std::string id;          // sorted key for binary search, e.g. "TGT-0001"
    std::string description;
    double importanceScore;  // higher = more scientifically valuable
};

// Quick sort descending by importanceScore. Average O(n log n);
// random pivot selection avoids the O(n^2) worst case on sorted/adversarial input.
void quickSortByImportance(std::vector<ExplorationTarget>& targets, int low, int high) {
    if (low >= high) return;

    static std::mt19937 rng(std::random_device{}());
    std::uniform_int_distribution<int> dist(low, high);
    std::swap(targets[dist(rng)], targets[high]);

    double pivot = targets[high].importanceScore;
    int i = low - 1;
    for (int j = low; j < high; ++j) {
        if (targets[j].importanceScore > pivot) {  // descending
            ++i;
            std::swap(targets[i], targets[j]);
        }
    }
    std::swap(targets[i + 1], targets[high]);
    int pivotIndex = i + 1;

    quickSortByImportance(targets, low, pivotIndex - 1);
    quickSortByImportance(targets, pivotIndex + 1, high);
}

void quickSortByImportance(std::vector<ExplorationTarget>& targets) {
    if (!targets.empty())
        quickSortByImportance(targets, 0, static_cast<int>(targets.size()) - 1);
}

// ---------------------------------------------------------------------------
// 3. BINARY SEARCH — Instant lookup of past research records
// ---------------------------------------------------------------------------
struct ResearchRecord {
    std::string id;  // sorted key, e.g. "REC-0042"
    std::string summary;
};

class ResearchArchive {
public:
    // Keeps records_ sorted by id for O(log n) lookups; insert is O(n),
    // the right trade-off since the archive is read far more than written.
    void addRecord(const ResearchRecord& record) {
        auto it = std::lower_bound(
            records_.begin(), records_.end(), record,
            [](const ResearchRecord& a, const ResearchRecord& b) { return a.id < b.id; });
        records_.insert(it, record);
    }

    std::optional<ResearchRecord> lookup(const std::string& id) const {
        int lo = 0, hi = static_cast<int>(records_.size()) - 1;
        while (lo <= hi) {
            int mid = lo + (hi - lo) / 2;
            if (records_[mid].id == id) return records_[mid];
            if (records_[mid].id < id) lo = mid + 1;
            else hi = mid - 1;
        }
        return std::nullopt;
    }

private:
    std::vector<ResearchRecord> records_;
};

// ---------------------------------------------------------------------------
// 4 & 5. GRAPH — Dijkstra for cheapest route, BFS for connectivity check
// ---------------------------------------------------------------------------
class UnderwaterGraph {
public:
    // Weighted, undirected edge (e.g. travel cost/time/energy between nodes).
    void addRoute(const std::string& a, const std::string& b, double weight) {
        adj_[a].push_back({b, weight});
        adj_[b].push_back({a, weight});
    }

    // Dijkstra's algorithm: like a GPS comparing routes by total travel
    // cost rather than number of hops — it always expands the cheapest
    // frontier node next, guaranteeing the lowest total-cost path once a
    // node is finalized. Requires non-negative weights. O(E log V) with a
    // binary min-heap (std::priority_queue).
    std::optional<std::pair<std::vector<std::string>, double>> dijkstraShortestRoute(
        const std::string& start, const std::string& target) const {
        std::unordered_map<std::string, double> dist;
        std::unordered_map<std::string, std::string> prev;
        using PQItem = std::pair<double, std::string>;  // (distance, node)
        std::priority_queue<PQItem, std::vector<PQItem>, std::greater<>> pq;

        dist[start] = 0.0;
        pq.push({0.0, start});

        while (!pq.empty()) {
            auto [d, node] = pq.top();
            pq.pop();

            if (dist.count(node) && d > dist[node]) continue;  // stale entry
            if (node == target) break;

            auto it = adj_.find(node);
            if (it == adj_.end()) continue;

            for (const auto& [neighbor, weight] : it->second) {
                double newDist = d + weight;
                if (!dist.count(neighbor) || newDist < dist[neighbor]) {
                    dist[neighbor] = newDist;
                    prev[neighbor] = node;
                    pq.push({newDist, neighbor});
                }
            }
        }

        if (!dist.count(target)) return std::nullopt;  // unreachable

        std::vector<std::string> path;
        std::string cur = target;
        while (cur != start) {
            path.push_back(cur);
            cur = prev[cur];
        }
        path.push_back(start);
        std::reverse(path.begin(), path.end());

        return std::make_pair(path, dist[target]);
    }

    // BFS: verifies every vehicle node can reach a research station,
    // ignoring edge weights entirely — connectivity is a yes/no question,
    // so the cheaper unweighted traversal is the right tool. O(V + E).
    bool bfsAllConnected(const std::vector<std::string>& vehicles,
                          const std::string& station) const {
        for (const auto& vehicle : vehicles) {
            if (!bfsReachable(vehicle, station)) return false;
        }
        return true;
    }

    std::vector<std::string> bfsUnreachableVehicles(
        const std::vector<std::string>& vehicles, const std::string& station) const {
        std::vector<std::string> unreachable;
        for (const auto& vehicle : vehicles) {
            if (!bfsReachable(vehicle, station)) unreachable.push_back(vehicle);
        }
        return unreachable;
    }

private:
    struct Edge {
        std::string to;
        double weight;
    };
    std::unordered_map<std::string, std::vector<Edge>> adj_;

    bool bfsReachable(const std::string& start, const std::string& target) const {
        if (start == target) return true;
        std::unordered_set<std::string> visited{start};
        std::queue<std::string> q;
        q.push(start);

        while (!q.empty()) {
            std::string node = q.front();
            q.pop();
            auto it = adj_.find(node);
            if (it == adj_.end()) continue;
            for (const auto& edge : it->second) {
                if (edge.to == target) return true;
                if (!visited.count(edge.to)) {
                    visited.insert(edge.to);
                    q.push(edge.to);
                }
            }
        }
        return false;
    }
};

// ---------------------------------------------------------------------------
// DEMO
// ---------------------------------------------------------------------------
int main() {
    std::cout << std::string(70, '=') << "\n";
    std::cout << "1. QUEUE - Incoming sensor data processed in order\n";
    std::cout << std::string(70, '=') << "\n";
    SensorQueue sensors;
    sensors.enqueue({"AUV-1", "temperature spike detected"});
    sensors.enqueue({"AUV-2", "coral formation detected"});
    sensors.enqueue({"AUV-3", "routine telemetry"});
    for (const auto& line : sensors.processAll()) {
        std::cout << "  " << line << "\n";
    }

    std::cout << "\n" << std::string(70, '=') << "\n";
    std::cout << "2. QUICK SORT - Rank exploration targets by importance\n";
    std::cout << std::string(70, '=') << "\n";
    std::vector<ExplorationTarget> targets = {
        {"TGT-0003", "Unusual coral formation", 9.2},
        {"TGT-0001", "Hydrothermal vent field", 8.7},
        {"TGT-0004", "Shipwreck debris field", 4.1},
        {"TGT-0002", "Deep trench sediment sample", 6.5},
    };
    quickSortByImportance(targets);
    for (size_t i = 0; i < targets.size(); ++i) {
        std::cout << "  #" << (i + 1) << ": " << targets[i].description
                   << " (score " << targets[i].importanceScore << ")\n";
    }

    std::cout << "\n" << std::string(70, '=') << "\n";
    std::cout << "3. BINARY SEARCH - Past research record lookup\n";
    std::cout << std::string(70, '=') << "\n";
    ResearchArchive archive;
    archive.addRecord({"REC-0042", "2024 survey of hydrothermal vent field"});
    archive.addRecord({"REC-0007", "Coral bleaching baseline study"});
    archive.addRecord({"REC-0099", "Trench sediment composition report"});

    for (const auto& query : {std::string("REC-0042"), std::string("REC-1234")}) {
        auto result = archive.lookup(query);
        if (result) {
            std::cout << "  Found " << query << ": " << result->summary << "\n";
        } else {
            std::cout << "  " << query << ": not found\n";
        }
    }

    std::cout << "\n" << std::string(70, '=') << "\n";
    std::cout << "4. DIJKSTRA'S ALGORITHM - Cheapest route to a target site\n";
    std::cout << std::string(70, '=') << "\n";
    UnderwaterGraph grid;
    grid.addRoute("Station-A", "Waypoint-1", 4.0);
    grid.addRoute("Station-A", "Waypoint-2", 2.0);
    grid.addRoute("Waypoint-2", "Waypoint-1", 1.0);
    grid.addRoute("Waypoint-1", "CoralSite", 5.0);
    grid.addRoute("Waypoint-2", "Waypoint-3", 6.0);
    grid.addRoute("Waypoint-3", "CoralSite", 1.0);

    std::cout << "  Scenario: AUV-2 found a coral formation, needs the cheapest route there.\n";
    auto result = grid.dijkstraShortestRoute("Station-A", "CoralSite");
    if (result) {
        auto& [path, cost] = *result;
        std::cout << "  Cheapest route (cost " << cost << "): ";
        for (size_t i = 0; i < path.size(); ++i) {
            std::cout << path[i];
            if (i + 1 < path.size()) std::cout << " -> ";
        }
        std::cout << "\n";
    }
    // Note: Station-A -> Waypoint-1 -> CoralSite is the 2-hop route but
    // costs 9 (4 + 5). Dijkstra instead picks the 3-hop route via
    // Waypoint-2 which costs only 8 (2 + 1 + 5) — proof that "fewest
    // hops" (BFS) and "cheapest path" (Dijkstra) aren't the same thing
    // once edges carry different weights.

    std::cout << "\n" << std::string(70, '=') << "\n";
    std::cout << "5. BFS - Verify all vehicles stay connected to a research station\n";
    std::cout << std::string(70, '=') << "\n";
    std::vector<std::string> vehicles = {"Waypoint-1", "Waypoint-3", "CoralSite"};
    bool allConnected = grid.bfsAllConnected(vehicles, "Station-A");
    std::cout << "  All vehicles connected to Station-A? "
               << (allConnected ? "YES" : "NO") << "\n";

    // Demonstrate a disconnected vehicle
    UnderwaterGraph gridWithIsolation = grid;
    gridWithIsolation.addRoute("Rogue-AUV", "IsolatedWaypoint", 3.0);
    std::vector<std::string> vehiclesWithRogue = {"Waypoint-1", "Rogue-AUV"};
    auto unreachable = gridWithIsolation.bfsUnreachableVehicles(vehiclesWithRogue, "Station-A");
    std::cout << "  Vehicles NOT connected to Station-A: ";
    if (unreachable.empty()) {
        std::cout << "none\n";
    } else {
        for (size_t i = 0; i < unreachable.size(); ++i) {
            std::cout << unreachable[i];
            if (i + 1 < unreachable.size()) std::cout << ", ";
        }
        std::cout << "\n";
    }

    return 0;
}
