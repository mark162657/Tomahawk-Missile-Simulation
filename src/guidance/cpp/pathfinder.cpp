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
        std::vector<float> g_score(m_total_pixels, std::numeric_limits<float>::infinity())
        std::vector<int> came_from(m_totao_pixels, -1) // fill the array with -1 (None, Null...) first

        // Initiate g_score
        g_score[start_idx] = 0.0f;
        std::priority_queue<Node, >

    }




private:



};
int main() {
}