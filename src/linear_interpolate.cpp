#include <vector>
#include <stdexcept>

float linear_interpolate(const std::vector<float>& x_vals, const std::vector<float>& y_vals, float x) {
    if (x_vals.size() != y_vals.size() || x_vals.empty()) {
        throw std::invalid_argument("Input vectors must be of equal non-zero length.");
    }

    // Handle extrapolation cases
    if (x <= x_vals.front()) return y_vals.front();
    if (x >= x_vals.back()) return y_vals.back();

    // Find the interval x is in
    for (size_t i = 0; i < x_vals.size() - 1; ++i) {
        if (x >= x_vals[i] && x <= x_vals[i + 1]) {
            float t = (x - x_vals[i]) / (x_vals[i + 1] - x_vals[i]);
            return y_vals[i] + t * (y_vals[i + 1] - y_vals[i]);
        }
    }

    // Should not reach here if inputs are correct
    throw std::runtime_error("Interpolation failed.");
}
