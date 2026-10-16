#pragma once

#include <utility>
#include <vector>

namespace osm {

	struct OSM_Node {
		uint64_t osm_id;
		double longitude, latitude;
	};

	struct OSM_Way {
		std::vector<uint64_t> nodes;
		uint32_t unique_name_index = 0;
		bool oneway = false;
		uint8_t laneCount = 2;
		uint8_t typeOfRoad = 0;
	};

#pragma pack(push, 1)
	struct Packed_Node {
		uint16_t x;
		uint16_t y;
		float elevation = 0.0f;
	};
#pragma pack(pop)

#pragma pack(push, 1)
	// 12 bytes
	struct Packed_Way_Header {
		uint32_t first_node_index;
		uint16_t name_idx;
		uint8_t nr_of_nodes;
		uint8_t neighbour_chunk_id;
		uint8_t type_of_way;
		uint8_t bucket0;
		uint8_t bucket1;
		uint8_t bucket2;
	};
#pragma pack(pop)

	struct ChunkIndex {
		int X = -1, Y = -1;

		bool operator==(const ChunkIndex& other) const {
			return X == other.X && Y == other.Y;
		}
	};

	template <int NumLayers>
	struct ChunkIndexArray {
		std::array<ChunkIndex, NumLayers> chk_idx_array;

		bool operator==(const ChunkIndexArray& other) const {
			return chk_idx_array == other.chk_idx_array;
		}
	};

}