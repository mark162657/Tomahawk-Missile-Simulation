#include <iostream>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <vector>
#include<queue>
#include<cmath>

namespace py = pybind11;

// Setting up priority queue Node (heap)
struct Node {
    float f_score;
    int index;

    // Min-heap comparator, lowest on top
    // Sort node based on f_score
    bool operator>(const Node& other) const {
        return f_score > other.f_score;
    }
};


class PathFinderCPP {
public:
    // Constructor: accepting numpy array and var from Python
    PathFinderCPP(
        py::array_t<float> dem_array, // float: float32
        py::array<double> lat_lookup_table, // double: float64
        int rows,
        int cols,
        double meter_per_z)
        : m_rows(rows), m_cols(cols), m_meter_per_z(meter_per_z) {
            auto buff_dem = dem_array.request();
            auto buff_lat = lat_lookup_table.request();

            // Pointer for raw access
            ptr_dem = static_cast<float*>(buffer_dem.ptr);
            ptr_lat_lookup = static_cast<double*>(buffer_lat.ptr);

            m_total_pixels = m_rows * m_cols;
            std::cout << "C++ Engine Online. Map Size: " << m_rows << "x" << m_cols
                << " (" << m_total_pixels << " pixels)" << std::endl;
    }
    // --- Main A* Pathfinding Algorithm ---
    // std::pair in std::vector == storing tuple() in py list
    std::vector<std::pair<int, int>> find_path(int start_idx, int end_idx, float heuristic_weight=1) {
        // Bound check
        if (start_idx < 0 || start_idx >= m_total_pixels || end_idx < 0 || end_idx >= m_total_pixels) {
            std::cerr << "[C++] Error: Start or End index out of bounds!" << std::endl;
            return {};
        }
        // Allocating dense memory for our arrays
        std::vector<float> g_score(m_total_pixels, std::numeric_limits<float>::infinity());
        std::vector<int> came_from(m_total_pixels, -1); // fill the array with -1 (None, Null...) first

        // Initiate g_score
        g_score[start_idx] = 0.0f;
        std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_set;
        open_set.push({0.0f, start_idx});

        long node_explored = 0;

        while (!open_set.empty()) {

        }
    }

private:
    float* ptr_dem;
    double* ptr_lat_lookup_table;

    double m_meter_per_z;
    int m_rows;
    int m_cols;
    int m_total_pixels;

    // Helper functions (using inline to speed up)
    // --- _heuristic ---
    inline float heuristic(int idx1, int idx2) {
        int row1 = idx1 / m_cols; int col1 = idx1 % m_cols;
        int row2 = idx2 / m_cols; int col2 = idx2 % m_cols;

        float dist_z = std::abs(row2 - row1) * m_meter_per_z;
        int midrow = (row1 + row2) / 2;
        double dist_x = std::abs(col2 - col1) * ptr_lat_lookup_table[midrow];

        return (float) std::sqrt(dist_x * dist_x + dist_z * dist_z);
    }

    // --- _get_movement_cost ---
    inline float get_movement_cost(int current_idx, int neighbor_idx) {

        float current_elev = ptr_dem[current_idx];
        float neighbor_elev = ptr_dem[neighbor_idx];

        // No-data check
        if (current_elev <= -100.0f || neighbor_elev <= -100.0f) return std::numeric_limits<float>::infinity();

        // Base distances: recalculate heuristic here to avoid function call cost
        // yes, so extract every penny as possible lol, for handling at least 800 million pixels
        int row1 = current_idx / m_cols; int col1 = current_idx % m_cols;
        int row2 = neighbor_idx / m_cols; int col2 = neighbor_idx % m_cols;

        double dist_z = std::abs(row2 - row1) * m_meter_per_z;
        double dist_x = std::abs(col2 - col1) * ptr_lat_lookup_table[row1];
        float dist_cost = std::sqrt((dist_x * dist_x) + (dist_z * dist_z));

        // !! Crucial !! Apply extra serious penalty on altitude change, otherwise the algorithm will still choose
        // short distance vs low-altitude terrain
        // Note: This is also the part which results in the significant increase in compile time (at least in Python)
        double height_penalty = neighbor_elev * 0.5;

        // --- Handle: decision penalties ---
        float climb = neighbor_elev - current_elev;

        // downhill
        if (climb < 0) return dist_cost + (climb * 0.5) + height_penalty;

        // uphill
        float gradient = climb / (dist_cost + 1e-6f);
        float penalty = 0.0f;

        // Level 1: gentle slope (<3°)
        if (gradient <= 0.05f) {
            // ignore, penalty weight: low (scale of 2)
            penalty = climb * 2.0f;
        }
        // Level 2: moderate (3-8.5°)
        else if (gradient <= 0.15f) {
            // penalty weight: medium (10)
            penalty = climb * 10.0f;
        }
        // Level 3: steep (>8.5°)
        else {
            // penalty weight: high (100), the program will choose other path unless still the closest even with penalty
            penalty = climb * 100.0f; // apply a lot of penalty to very steep
        }
        return dist_cost + penalty + height_penalty;
    }

    // --- _reconstruct_path ---
    std::vector<std::pair<int, int>> reconstruct_path(const std::vector<int> $came_from, int current_idx=0) {
        std::vector<std::pair<int, int>> path;

        while (current_idx != -1) {
            int row = current_idx / m_cols;
            int col = current_idx % m_cols;

            path.push_back({row, col});

        }
        std::reverse(path.begin(), path.end());
        return path;

    }

};
int main() {
}