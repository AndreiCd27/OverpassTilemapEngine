#pragma once

#include "GeoUtility.h"
#include "OsmTypes.h"
#include <vector>
#include <array>

using transformer = geo::Stereo70Transformer;
using bbox = geo::Stereo70BBox;
inline auto map_bounds = geo::ro::ro_bounds_m;
inline auto map_size = geo::ro::MapSize_m;

struct ChunkMetadata {
	int Layer;
	int i, j;
	bbox Bounds_m;
};

class ChunkManager {

	ChunkMetadata MapLayer;
	const uint32_t divChunk = 4;

public:

	ChunkManager(uint32_t divChunk) : divChunk(divChunk) {
		MapLayer = ChunkMetadata{
			.Layer = 0, .i = 0, .j = 0, .Bounds_m = map_bounds
		};
	}

	ChunkMetadata MakeChunk_at(int i, int j, const ChunkMetadata* src_chk_meta) const;

	const ChunkMetadata& GetMapLayer() const {
		return MapLayer;
	}

	template <int NumLayers>
	ChunkMetadata GetLeafMetadata(const osm::ChunkIndexArray<NumLayers>& indices) const {

		ChunkMetadata current_meta = MapLayer;

		for (int l = 0; l < NumLayers; l++) {
			int target_i = indices.chk_idx_array[l].X;
			int target_j = indices.chk_idx_array[l].Y;
			current_meta = MakeChunk_at(target_i, target_j, &current_meta);
		}

		return current_meta;
	}
};
