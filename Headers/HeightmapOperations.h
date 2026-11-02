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

		using json = nlohmann::json;

		// Here we parse the json response from local OpenTopoData API

		const int res = 16;

		std::vector<std::vector<float>> matrix(res, std::vector(res, 0.0f));

		try {
			json data = json::parse(json_response);

			if (data.contains("results") && data["results"].is_array()) {

				auto& results_array = data["results"];
				int index = 0;

				for (int i = 0; i < res; i++) {
					for (int j = 0; j < res; j++) {
						
						if (index < results_array.size()) {

							float elevation = results_array[index].value("elevation", 0.0f);

							matrix[i][j] = elevation;
							index++;
						}
					}
				}
			}
			else {
				std::cout << "Invalid JSON response (Error)" << std::endl;
				std::cout << json_response << std::endl;
			}
		}
		catch (const json::parse_error& e) {
			std::cerr << "Erorr parsing JSON response from OpenTopoData: " << e.what() << "\n";
			std::cout << json_response << std::endl;
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
