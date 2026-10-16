
#include "ChunkManager.h"

ChunkMetadata ChunkManager::MakeChunk_at(int i, int j, const ChunkMetadata* src_chk_meta) const {
    
    const ChunkMetadata& source = (src_chk_meta == nullptr) ? MapLayer : *src_chk_meta;

    int target_chunk_size = map_size;
    for (int step = 0; step < source.Layer + 1; ++step) {
        target_chunk_size /= divChunk;
    }

    ChunkMetadata dst_chk_meta{
        .Layer = source.Layer + 1,
        .i = i,
        .j = j,
        .Bounds_m = bbox{
            .x0 = source.Bounds_m.x0 + target_chunk_size * i,
            .y0 = source.Bounds_m.y0 + target_chunk_size * j,
            .x1 = source.Bounds_m.x0 + target_chunk_size * (i + 1),
            .y1 = source.Bounds_m.y0 + target_chunk_size * (j + 1)
        }
    };

    return dst_chk_meta;
}