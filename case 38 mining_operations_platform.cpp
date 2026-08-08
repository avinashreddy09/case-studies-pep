/*
 * Smart Mining Operations Optimization Platform
 * ================================================
 * Case Study #38 — Full C++ implementation of all five components:
 *
 *   1. Queue        -> real-time machine event processing         O(1)
 *   2. Heap Sort     -> ranking mining zones by profitability      O(n log n)
 *   3. Binary Search  -> instant inventory lookups                 O(log n)
 *   4. BFS            -> fastest emergency evacuation route        O(V + E)
 *   5. DFS             -> deep investigation of a specific tunnel   O(V + E)
 *
 * Compile:  g++ -std=c++17 -O2 -o mining mining_operations_platform.cpp
 * Run:      ./mining
 */

#include <iostream>
#include <vector>
#include <queue>
#include <deque>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <optional>
#include <sstream>
#include <functional>

// ---------------------------------------------------------------------------
// 1. QUEUE — Machine-generated event processing
// ---------------------------------------------------------------------------
struct Event {
    std::string type;  // "gas_leak" | "equipment_fault" | "sensor_ping"
    std::string zone;
};

class EventQueue {
public:
    void enqueue(const Event& e) { queue_.push(e); }

    std::optional<Event> dequeue() {
        if (queue_.empty()) return std::nullopt;
        Event e = queue_.front();
        queue_.pop();
        return e;
    }

    // Drain the queue, handling each event in arrival order (FIFO).
    std::vector<std::string> processAll() {
        std::vector<std::string> log;
        while (auto e = dequeue()) {
            log.push_back(handle(*e));
        }
        return log;
    }

    size_t size() const { return queue_.size(); }

private:
    std::queue<Event> queue_;

    static std::string handle(const Event& e) {
        if (e.type == "gas_leak")
            return "[ALERT] Gas leak detected in " + e.zone + " — triggering evacuation protocol.";
        if (e.type == "equipment_fault")
            return "[WARN] Equipment fault reported in " + e.zone + ".";
        if (e.type == "sensor_ping")
            return "[OK] Sensor heartbeat received from " + e.zone + ".";
        return "[INFO] Unrecognized event from " + e.zone + ": " + e.type;
    }
};

// ---------------------------------------------------------------------------
// 2. HEAP SORT — Rank mining zones by profitability
// ---------------------------------------------------------------------------
struct Zone {
    std::string name;
    double profitability;  // e.g. estimated $/ton of ore
};

// Standard heap sort, O(n log n) time, O(1) extra space (in-place).
// descending = true -> most profitable zone ends up first.
void heapSortZones(std::vector<Zone>& zones, bool descending = true) {
    int n = static_cast<int>(zones.size());

    auto better = [&](const Zone& a, const Zone& b) {
        return descending ? (a.profitability > b.profitability)
                           : (a.profitability < b.profitability);
    };

    std::function<void(int, int)> siftDown = [&](int i, int size) {
        int best = i;
        int left = 2 * i + 1, right = 2 * i + 2;
        if (left < size && better(zones[left], zones[best])) best = left;
        if (right < size && better(zones[right], zones[best])) best = right;
        if (best != i) {
            std::swap(zones[i], zones[best]);
            siftDown(best, size);
        }
    };

    // Build max-heap
    for (int i = n / 2 - 1; i >= 0; --i) siftDown(i, n);

    // Extract root repeatedly to the end, shrinking the heap
    for (int end = n - 1; end > 0; --end) {
        std::swap(zones[0], zones[end]);
        siftDown(0, end);
    }

    // siftDown leaves the array in ascending "better" order; reverse it
    // so the most profitable zone is first when descending = true.
    std::reverse(zones.begin(), zones.end());
}

// ---------------------------------------------------------------------------
// 3. BINARY SEARCH — Instant inventory lookup
// ---------------------------------------------------------------------------
struct InventoryItem {
    std::string partId;  // sorted key, e.g. "DRILL-0042"
    std::string name;
    int quantity;
};

class InventorySystem {
public:
    // Insert while keeping items_ sorted by partId (O(n) insert,
    // O(log n) lookup — the right trade-off when reads vastly
    // outnumber writes, as with field crews querying constantly).
    void addItem(const InventoryItem& item) {
        auto it = std::lower_bound(
            items_.begin(), items_.end(), item,
            [](const InventoryItem& a, const InventoryItem& b) {
                return a.partId < b.partId;
            });
        items_.insert(it, item);
    }

    std::optional<InventoryItem> lookup(const std::string& partId) const {
        int lo = 0, hi = static_cast<int>(items_.size()) - 1;
        while (lo <= hi) {
            int mid = lo + (hi - lo) / 2;
            if (items_[mid].partId == partId) return items_[mid];
            if (items_[mid].partId < partId) lo = mid + 1;
            else hi = mid - 1;
        }
        return std::nullopt;
    }

private:
    std::vector<InventoryItem> items_;
};

// ---------------------------------------------------------------------------
// 4 & 5. GRAPH — BFS for shortest evacuation route, DFS for full path trace
// ---------------------------------------------------------------------------
class TunnelGraph {
public:
    void addTunnel(const std::string& a, const std::string& b) {
        adj_[a].push_back(b);
        adj_[b].push_back(a);
    }

    // Breadth-first search: explores the graph one "ring" of distance at a
    // time (like ripples from a stone dropped in water), so the first time
    // `surface` is reached it's guaranteed to be via the fewest possible
    // tunnel segments — the fastest evacuation route in an unweighted
    // graph. O(V + E).
    std::optional<std::vector<std::string>> bfsShortestEvacuation(
        const std::string& start, const std::string& surface) const {
        if (start == surface) return std::vector<std::string>{start};

        std::unordered_set<std::string> visited{start};
        std::queue<std::vector<std::string>> q;
        q.push({start});

        while (!q.empty()) {
            auto path = q.front();
            q.pop();
            const std::string& node = path.back();

            auto it = adj_.find(node);
            if (it == adj_.end()) continue;

            for (const auto& neighbor : it->second) {
                if (visited.count(neighbor)) continue;
                auto newPath = path;
                newPath.push_back(neighbor);
                if (neighbor == surface) return newPath;
                visited.insert(neighbor);
                q.push(newPath);
            }
        }
        return std::nullopt;  // no path to surface
    }

    // Depth-first search: follows one tunnel branch as deep as it goes
    // before backtracking. Useful for inspecting a specific path in full
    // (e.g. tracing structural damage along one route), NOT for
    // shortest-path guarantees. O(V + E).
    std::optional<std::vector<std::string>> dfsInvestigateTunnel(
        const std::string& start, const std::string& target) const {
        std::unordered_set<std::string> visited;
        std::vector<std::string> path;

        std::function<bool(const std::string&)> dfs = [&](const std::string& node) -> bool {
            visited.insert(node);
            path.push_back(node);
            if (node == target) return true;

            auto it = adj_.find(node);
            if (it != adj_.end()) {
                for (const auto& neighbor : it->second) {
                    if (!visited.count(neighbor) && dfs(neighbor)) return true;
                }
            }
            path.pop_back();
            return false;
        };

        if (dfs(start)) return path;
        return std::nullopt;
    }

private:
    std::unordered_map<std::string, std::vector<std::string>> adj_;
};

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
std::string joinPath(const std::vector<std::string>& path) {
    std::ostringstream oss;
    for (size_t i = 0; i < path.size(); ++i) {
        oss << path[i];
        if (i + 1 < path.size()) oss << " -> ";
    }
    return oss.str();
}

// ---------------------------------------------------------------------------
// DEMO
// ---------------------------------------------------------------------------
int main() {
    std::cout << std::string(70, '=') << "\n";
    std::cout << "1. QUEUE - Machine event processing\n";
    std::cout << std::string(70, '=') << "\n";
    EventQueue events;
    events.enqueue({"sensor_ping", "Zone-A"});
    events.enqueue({"equipment_fault", "Zone-C"});
    events.enqueue({"gas_leak", "Zone-B"});
    for (const auto& line : events.processAll()) {
        std::cout << "  " << line << "\n";
    }

    std::cout << "\n" << std::string(70, '=') << "\n";
    std::cout << "2. HEAP SORT - Zone profitability ranking\n";
    std::cout << std::string(70, '=') << "\n";
    std::vector<Zone> zones = {
        {"Zone-A", 125000},
        {"Zone-B", 480500},
        {"Zone-C", 92300},
        {"Zone-D", 310750},
        {"Zone-E", 205000},
    };
    heapSortZones(zones, true);
    for (size_t i = 0; i < zones.size(); ++i) {
        std::cout << "  #" << (i + 1) << ": " << zones[i].name
                   << " ($" << zones[i].profitability << ")\n";
    }

    std::cout << "\n" << std::string(70, '=') << "\n";
    std::cout << "3. BINARY SEARCH - Inventory lookup\n";
    std::cout << std::string(70, '=') << "\n";
    InventorySystem inventory;
    inventory.addItem({"DRILL-0042", "Rotary Drill Bit", 18});
    inventory.addItem({"HELM-0007", "Safety Helmet", 240});
    inventory.addItem({"OXYG-0099", "Oxygen Tank", 56});
    inventory.addItem({"CART-0015", "Mining Cart Wheel", 73});

    for (const auto& query : {std::string("OXYG-0099"), std::string("DRILL-9999")}) {
        auto result = inventory.lookup(query);
        if (result) {
            std::cout << "  Found " << query << ": " << result->name
                       << ", qty=" << result->quantity << "\n";
        } else {
            std::cout << "  " << query << ": not found\n";
        }
    }

    std::cout << "\n" << std::string(70, '=') << "\n";
    std::cout << "4 & 5. GRAPH - Evacuation (BFS) and tunnel investigation (DFS)\n";
    std::cout << std::string(70, '=') << "\n";
    TunnelGraph tunnels;
    tunnels.addTunnel("Zone-B", "Junction-1");
    tunnels.addTunnel("Junction-1", "Junction-2");
    tunnels.addTunnel("Junction-1", "Junction-3");
    tunnels.addTunnel("Junction-2", "Junction-4");
    tunnels.addTunnel("Junction-3", "Surface");
    tunnels.addTunnel("Junction-4", "Surface");
    tunnels.addTunnel("Zone-B", "Junction-5");
    tunnels.addTunnel("Junction-5", "Junction-6");

    std::cout << "  Scenario: Gas leak detected in Zone-B, workers must reach Surface.\n";
    auto route = tunnels.bfsShortestEvacuation("Zone-B", "Surface");
    if (route) std::cout << "  BFS fastest evacuation route: " << joinPath(*route) << "\n";

    std::cout << "\n  Investigation: trace a full path from Zone-B into Junction-6\n";
    auto trace = tunnels.dfsInvestigateTunnel("Zone-B", "Junction-6");
    if (trace) std::cout << "  DFS path traced: " << joinPath(*trace) << "\n";

    return 0;
}
