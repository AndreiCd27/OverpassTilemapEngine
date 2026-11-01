#pragma once

#include <vector>
#include <cmath>
#include <algorithm>

namespace FFT2D {

    constexpr float PI = 3.14159f;

#pragma pack(push, 1)
    struct FourierTerm {
        float amplitude;
        uint16_t u; // Frequency index along X
        uint16_t v; // Frequency index along Z
    };
#pragma pack(pop)

    // Computes the 2D DCT-II and extracts the dominant coefficients into out_terms
    static void compute2DFourier(const std::vector<std::vector<float>>& heightmap, 
        int maxTerms, std::vector<FourierTerm>& out_terms) {

        int res = heightmap.size(); // Grid resolution

        // Compute 2D DCT
        for (int u = 0; u < res; ++u) {
            for (int v = 0; v < res; ++v) {
                // Skip high frequencies
                if (u + v > 32) continue;

                float sum = 0.0f;
                for (int x = 0; x < res; ++x) {
                    for (int z = 0; z < res; ++z) {
                        sum += heightmap[x][z] *
                            std::cos((PI / res) * (x + 0.5f) * u) *
                            std::cos((PI / res) * (z + 0.5f) * v);
                    }
                }

                // Normalization factors for DCT-II
                float cu = (u == 0) ? std::sqrt(1.0f / res) : std::sqrt(2.0f / res);
                float cv = (v == 0) ? std::sqrt(1.0f / res) : std::sqrt(2.0f / res);
                float amplitude = sum * cu * cv;

                // Only store terms that actually contribute to the shape
                if (std::abs(amplitude) > 0.01f) {
                    FourierTerm ft; ft.amplitude = amplitude; ft.u = u; ft.v = v;
                    out_terms.push_back(ft);
                }
            }
        }

        // Sort terms by absolute amplitude descending to find the most dominant waves
        std::sort(out_terms.begin(), out_terms.end(), [](const FourierTerm& a, const FourierTerm& b) {
            return std::abs(a.amplitude) > std::abs(b.amplitude);
            });

        if (out_terms.size() > maxTerms) {
            out_terms.resize(maxTerms);
        }
    }
}
