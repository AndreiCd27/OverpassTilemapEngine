#pragma once

#include <nlohmann/json.hpp>
#include <vector>
#include "FFT_2D.h"
#include "ChunkManager.h"


class Heightmap_Storage {

	using Heightmap_Buffer = std::vector<FFT2D::FourierTerm>;

	std::vector<Heightmap_Buffer> heightmaps;
	const int maxTerms;
	const int CHK_CNT;

public:

	Heightmap_Storage(int maxTerms, int CHK_CNT) : maxTerms(maxTerms), CHK_CNT(CHK_CNT) {
		heightmaps.resize(CHK_CNT * CHK_CNT);
	}

	std::vector<std::vector<float>> ParseHeightmap(const std::string& json_response) {
		// Here we parse the json response from local OpenTopoData API

		// Return sample data for testing
		int res = 64;
		std::vector<std::vector<float>> matrix(res);
		for (int i = 0; i < res; i++) {
			std::vector<float>& row = matrix[i];
			for (int j = 0; j < res; j++) {
				float pseudo_rand = float((i * 21783 + j * 2913) % 1781) / 401.0f;
				matrix[i].push_back(pseudo_rand);
			}
		}

		return matrix;
	}

	void PackHeightmap(const std::vector<std::vector<float>>& matrix, int chk_x, int chk_y) {
		int idx1D = chk_x + chk_y * CHK_CNT;
		FFT2D::compute2DFourier(matrix, maxTerms, heightmaps[idx1D]);
	}

	void ComputeHeightmapForChunk(const std::string& json_response, int chk_x, int chk_y) {
		PackHeightmap(ParseHeightmap(json_response), chk_x, chk_y);
	}

	const Heightmap_Buffer& GetAtIndex(int idx1D) const {
		return heightmaps[idx1D];
	}
	Heightmap_Buffer& GetAtIndex_modifiable(int idx1D) {
		return heightmaps[idx1D];
	}
};
