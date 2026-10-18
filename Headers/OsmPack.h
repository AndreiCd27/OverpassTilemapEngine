#pragma once

#include "OsmParse.h"
#include "OsmTypes.h"
#include "ChunkManager.h"

#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include <filesystem>

namespace osm {

	template <int NumLayers>
	class OSM_Chunk_Layout {
		const ChunkManager& chk;
		transformer t; // geo::Stereo70Transformer pentru conversia WGS84 -> Stereo70
	public:
		OSM_Chunk_Layout(const ChunkManager& chunk_manager) : chk(chunk_manager) {}

		void GetChunksIndices(OSM_Node node, ChunkIndexArray<NumLayers>& out) {
			double x, y;
			t.wgs_to_stereo(node.latitude, node.longitude, x, y);
			GetChunkIndices_fromXY(x, y, out);
		}

		void GetChunkIndices_fromXY(double x, double y, ChunkIndexArray<NumLayers>& out) {
			ChunkMetadata chk_meta = chk.GetMapLayer();
			auto Bounds = chk_meta.Bounds_m;

			if (!(Bounds.x0 < x && x < Bounds.x1 && Bounds.y0 < y && y < Bounds.y1)) {
				for (int l = 0; l < NumLayers; l++) {
					out.chk_idx_array[l] = ChunkIndex{ .X = -1, .Y = -1 };
				}
				return;
			}

			const int div = 16;

			for (int l = 0; l < NumLayers; l++) {
				double size_x = Bounds.x1 - Bounds.x0;
				double size_y = Bounds.y1 - Bounds.y0;

				double dx = x - Bounds.x0;
				double dy = y - Bounds.y0;

				int idxX = static_cast<int>((dx / size_x) * div);
				int idxY = static_cast<int>((dy / size_y) * div);

				if (idxX >= div) idxX = div - 1;
				if (idxY >= div) idxY = div - 1;
				if (idxX < 0)    idxX = 0;
				if (idxY < 0)    idxY = 0;

				ChunkIndex chk_idx{ .X = idxX, .Y = idxY };
				out.chk_idx_array[l] = chk_idx;

				chk_meta = chk.MakeChunk_at(chk_idx.X, chk_idx.Y, &chk_meta);
				Bounds = chk_meta.Bounds_m;
			}
		}
	};

	template <int NumLayers>
	class OSM_Packed_Storage {
		// Exactly 256 subchunks (16x16 grid) for the current active region
		std::vector<std::vector<Packed_Way_Header>> flat_ways_storage;
		std::vector<std::vector<Packed_Node>>       flat_nodes_storage;
		std::vector<std::vector<uint16_t>>          flat_connections_storage;

		// Track which global megachunk this storage context belongs to
		int current_megaX = -1;
		int current_megaZ = -1;

		inline uint32_t GetFlatIndex(const ChunkIndexArray<NumLayers>& indices) const {
			// Extract local subchunk coordinates (Layer 1)
			int subX = indices.chk_idx_array[1].X;
			int subZ = indices.chk_idx_array[1].Y;

			// Fallback guard
			if (subX < 0 || subX >= 16 || subZ < 0 || subZ >= 16) return 0xFFFFFFFF;

			// Local 1D index mapping directly between 0 and 255
			return (subZ << 4) | subX;
		}

	public:
		OSM_Packed_Storage(const OSM_Parser_Storage& parser_storage, const ChunkManager& chunk_manager, int megaX, int megaZ)
			: current_megaX(megaX), current_megaZ(megaZ)
		{
			// Pre-allocate exactly 256 entries
			flat_ways_storage.resize(256);
			flat_nodes_storage.resize(256);
			flat_connections_storage.resize(256);

			OSM_Chunk_Layout<NumLayers> layout(chunk_manager);

			const auto& global_nodes = parser_storage.Get__nodes_by_osm_id();
			const auto& global_ways = parser_storage.Get__ways();

			// Key is FlatChunkIndex, value is [OSM_ID -> Local_Index_In_Chunk]
			std::unordered_map<uint32_t, std::unordered_map<uint64_t, uint16_t>> node_global_to_local_idx;
			transformer t_node;

			// Pack nodes
			for (const auto& [node_id, osm_node] : global_nodes) {
				ChunkIndexArray<NumLayers> indices;
				layout.GetChunksIndices(osm_node, indices);

				// skip if node falls outside our targeted region completely
				if (indices.chk_idx_array[0].X != current_megaX || indices.chk_idx_array[0].Y != current_megaZ) {
					continue;
				}

				uint32_t idx1D = GetFlatIndex(indices);
				if (idx1D == 0xFFFFFFFF) continue;

				double stereo_x, stereo_y;
				t_node.wgs_to_stereo(osm_node.latitude, osm_node.longitude, stereo_x, stereo_y);

				ChunkMetadata leaf_meta = chunk_manager.GetLeafMetadata(indices);

				double norm_x = (stereo_x - leaf_meta.Bounds_m.x0) / (leaf_meta.Bounds_m.x1 - leaf_meta.Bounds_m.x0);
				double norm_y = (stereo_y - leaf_meta.Bounds_m.y0) / (leaf_meta.Bounds_m.y1 - leaf_meta.Bounds_m.y0);

				Packed_Node packed_node;
				packed_node.x = static_cast<uint16_t>(norm_x * 65535.0);
				packed_node.y = static_cast<uint16_t>(norm_y * 65535.0);
				packed_node.elevation = 0;
				packed_node.chk_offset_x = 0;
				packed_node.chk_offset_y = 0;

				auto& chunk_nodes = flat_nodes_storage[idx1D];
				uint16_t local_idx = static_cast<uint16_t>(chunk_nodes.size());
				chunk_nodes.push_back(packed_node);

				node_global_to_local_idx[idx1D][node_id] = local_idx;
			}

			// Pack ways (With chunk margin detection)
			for (const auto& osm_way : global_ways) {
				if (osm_way.nodes.size() < 2) continue;

				uint32_t current_chunk_idx = 0xFFFFFFFF;
				Packed_Way_Header* current_way_header = nullptr;

				int current_subX = -1;
				int current_subY = -1;

				for (size_t i = 0; i < osm_way.nodes.size(); ++i) {
					uint64_t global_node_id = osm_way.nodes[i];
					auto it_node = global_nodes.find(global_node_id);
					if (it_node == global_nodes.end()) continue;

					ChunkIndexArray<NumLayers> indices;
					layout.GetChunksIndices(it_node->second, indices);

					if (indices.chk_idx_array[0].X != current_megaX || indices.chk_idx_array[0].Y != current_megaZ) {
						current_chunk_idx = 0xFFFFFFFF;
						current_way_header = nullptr;
						continue;
					}

					uint32_t node_chunk_idx = GetFlatIndex(indices);
					if (node_chunk_idx == 0xFFFFFFFF) continue;

					int node_subX = indices.chk_idx_array[NumLayers - 1].X;
					int node_subY = indices.chk_idx_array[NumLayers - 1].Y;

					// Initialize first part of the way header
					if (current_chunk_idx == 0xFFFFFFFF) {
						current_chunk_idx = node_chunk_idx;
						current_subX = node_subX;
						current_subY = node_subY;

						Packed_Way_Header way_header;
						way_header.name_idx = static_cast<uint16_t>(osm_way.unique_name_index);
						way_header.type_of_way = osm_way.typeOfRoad;
						way_header.bucket0 = (osm_way.laneCount & 0x0F) | ((osm_way.oneway ? 1 : 0) << 4);
						way_header.bucket1 = 0;
						way_header.bucket2 = 0;
						way_header.bucket3 = 0;
						way_header.first_node_index = static_cast<uint32_t>(flat_connections_storage[current_chunk_idx].size());
						way_header.nr_of_nodes = 0;

						flat_ways_storage[current_chunk_idx].push_back(way_header);
						current_way_header = &flat_ways_storage[current_chunk_idx].back();
					}

					// Subchunk boundary crossing detection
					if (node_chunk_idx != current_chunk_idx) {
						uint32_t old_chunk_idx = current_chunk_idx;
						int old_subX = current_subX;
						int old_subY = current_subY;

						// Switch active chunk
						current_chunk_idx = node_chunk_idx;
						current_subX = node_subX;
						current_subY = node_subY;

						// Init a new way header for the continuation of our road in the next chunk
						Packed_Way_Header next_way_header;
						next_way_header.name_idx = static_cast<uint16_t>(osm_way.unique_name_index);
						next_way_header.type_of_way = osm_way.typeOfRoad;
						next_way_header.bucket0 = (osm_way.laneCount & 0x0F) | ((osm_way.oneway ? 1 : 0) << 4);
						next_way_header.bucket1 = 0;
						next_way_header.bucket2 = 0;
						next_way_header.bucket3 = 0;
						next_way_header.first_node_index = static_cast<uint32_t>(flat_connections_storage[current_chunk_idx].size());
						next_way_header.nr_of_nodes = 0;

						flat_ways_storage[current_chunk_idx].push_back(next_way_header);
						current_way_header = &flat_ways_storage[current_chunk_idx].back();

						// Here we connect nodes from different chunks
						uint64_t prev_global_id = osm_way.nodes[i - 1];
						uint16_t local_idx_in_old = node_global_to_local_idx[old_chunk_idx][prev_global_id];

						const Packed_Node& old_node = flat_nodes_storage[old_chunk_idx][local_idx_in_old];

						Packed_Node borrowed_node = old_node;

						borrowed_node.chk_offset_x = static_cast<int8_t>(old_subX - current_subX);
						borrowed_node.chk_offset_y = static_cast<int8_t>(old_subY - current_subY);

						// Add offseted node in the node array structure
						flat_nodes_storage[current_chunk_idx].push_back(borrowed_node);
						uint16_t new_local_idx = static_cast<uint16_t>(flat_nodes_storage[current_chunk_idx].size() - 1);

						node_global_to_local_idx[current_chunk_idx][prev_global_id] = new_local_idx;

						flat_connections_storage[current_chunk_idx].push_back(new_local_idx);
						current_way_header->nr_of_nodes++;
					}

					auto& local_node_map = node_global_to_local_idx[current_chunk_idx];
					auto it_local = local_node_map.find(global_node_id);

					// If current node is not yet in active chunk, look for it and clone it with chunk offset
					if (it_local == local_node_map.end()) {
						
						uint32_t native_chunk_idx = 0xFFFFFFFF;
						uint16_t native_local_idx = 0xFFFF;

						for (const auto& [c_idx, inner_map] : node_global_to_local_idx) {
							auto it_found = inner_map.find(global_node_id);
							if (it_found != inner_map.end()) {
								native_chunk_idx = c_idx;
								native_local_idx = it_found->second;
								break;
							}
						}

						if (native_chunk_idx != 0xFFFFFFFF) {
							// Calculate local offset and clone node
							const Packed_Node& native_node = flat_nodes_storage[native_chunk_idx][native_local_idx];

							// Calculate coordinates of native subchunk
							auto it_native_node = global_nodes.find(global_node_id);
							ChunkIndexArray<NumLayers> native_indices;
							layout.GetChunksIndices(it_native_node->second, native_indices);
							int native_subX = native_indices.chk_idx_array[NumLayers - 1].X;
							int native_subY = native_indices.chk_idx_array[NumLayers - 1].Y;

							Packed_Node borrowed_node = native_node;
							borrowed_node.chk_offset_x = static_cast<int8_t>(native_subX - current_subX);
							borrowed_node.chk_offset_y = static_cast<int8_t>(native_subY - current_subY);

							flat_nodes_storage[current_chunk_idx].push_back(borrowed_node);
							uint16_t new_local_idx = static_cast<uint16_t>(flat_nodes_storage[current_chunk_idx].size() - 1);

							local_node_map[global_node_id] = new_local_idx;
							flat_connections_storage[current_chunk_idx].push_back(new_local_idx);
						}
					}
					else {
						flat_connections_storage[current_chunk_idx].push_back(it_local->second);
					}

					current_way_header->nr_of_nodes++;
				}
			}
		}

		void WriteToDisk(const std::string& outputFolder, const NameBuffer& name_buffer) {
			std::filesystem::create_directories(outputFolder);

			// Save Global Name Buffer
			{
				std::string globalNamesFile = outputFolder + "/global_names.bin";
				std::ofstream file(globalNamesFile, std::ios::binary);
				if (file.is_open()) {
					char magic[4] = { 'R', 'O', 'M', 'N' };
					file.write(magic, 4);
					uint32_t fileVersion = 1;
					file.write(reinterpret_cast<char*>(&fileVersion), 4);

					uint16_t totalGlobalNames = static_cast<uint16_t>(name_buffer.size());
					std::vector<char> raw_string_block;
					for (int i = 0; i < totalGlobalNames; ++i) {
						std::string name = name_buffer.get_name(i);
						raw_string_block.insert(raw_string_block.end(), name.begin(), name.end());
						raw_string_block.push_back('\0');
					}

					uint32_t stringBlockSizeBytes = static_cast<uint32_t>(raw_string_block.size());
					file.write(reinterpret_cast<char*>(&totalGlobalNames), 2);
					file.write(reinterpret_cast<char*>(&stringBlockSizeBytes), 4);

					if (stringBlockSizeBytes > 0) {
						file.write(raw_string_block.data(), raw_string_block.size());
					}
					file.close();
				}
			}

			// Save the single active Megachunk file directly
			std::string fileName = outputFolder + "/megachunk_" + std::to_string(current_megaX) + "_" + std::to_string(current_megaZ) + ".bin";
			std::ofstream file(fileName, std::ios::binary);
			if (!file.is_open()) return;

			char magic[4] = { 'R', 'O', 'M', 'S' };
			file.write(magic, 4);
			uint32_t fileVersion = 1;
			file.write(reinterpret_cast<char*>(&fileVersion), 4);

			// Local Lookup Table (256 slots)
			std::vector<uint64_t> lookupTable(256, 0ULL);
			uint64_t lutPosition = file.tellp();
			file.write(reinterpret_cast<char*>(lookupTable.data()), lookupTable.size() * sizeof(uint64_t));

			// Serialize local array indices directly
			for (int subX = 0; subX < 16; subX++) {
				for (int subZ = 0; subZ < 16; subZ++) {

					uint32_t idx1D = (subZ << 4) | subX; // local 0..255 index

					const auto& nodes = flat_nodes_storage[idx1D];
					const auto& ways = flat_ways_storage[idx1D];
					const auto& connections = flat_connections_storage[idx1D];

					if (nodes.empty() && ways.empty()) continue;

					uint8_t lutIndex = ((subZ & 0xF) << 4) | (subX & 0xF);
					lookupTable[lutIndex] = static_cast<uint64_t>(file.tellp());

					uint32_t nodeCount = static_cast<uint32_t>(nodes.size());
					uint32_t wayCount = static_cast<uint32_t>(ways.size());
					uint32_t connCount = static_cast<uint32_t>(connections.size());

					file.write(reinterpret_cast<char*>(&nodeCount), 4);
					file.write(reinterpret_cast<char*>(&wayCount), 4);
					file.write(reinterpret_cast<char*>(&connCount), 4);

					if (nodeCount > 0) {
						file.write(reinterpret_cast<const char*>(nodes.data()), nodes.size() * sizeof(Packed_Node));
					}
					if (connCount > 0) {
						file.write(reinterpret_cast<const char*>(connections.data()), connections.size() * sizeof(uint16_t));
					}
					if (wayCount > 0) {
						file.write(reinterpret_cast<const char*>(ways.data()), ways.size() * sizeof(Packed_Way_Header));
					}
				}
			}

			file.seekp(lutPosition);
			file.write(reinterpret_cast<char*>(lookupTable.data()), lookupTable.size() * sizeof(uint64_t));
			file.close();
		}

		bool ReadFromDisk(const std::string& inputFolder, NameBuffer& out_global_name_buffer) {
			// Clear and resize structures to local grid size (256)
			flat_ways_storage.assign(256, std::vector<Packed_Way_Header>());
			flat_nodes_storage.assign(256, std::vector<Packed_Node>());
			flat_connections_storage.assign(256, std::vector<uint16_t>());

			// Unpack Names
			std::string globalNamesFile = inputFolder + "/global_names.bin";
			if (std::filesystem::exists(globalNamesFile)) {
				std::ifstream file(globalNamesFile, std::ios::binary);
				if (file.is_open()) {
					char magic[4]; file.read(magic, 4);
					uint32_t fileVersion = 0; file.read(reinterpret_cast<char*>(&fileVersion), 4);
					uint16_t totalGlobalNames = 0;
					uint32_t stringBlockSizeBytes = 0;
					file.read(reinterpret_cast<char*>(&totalGlobalNames), 2);
					file.read(reinterpret_cast<char*>(&stringBlockSizeBytes), 4);

					if (stringBlockSizeBytes > 0) {
						std::vector<char> raw_string_block(stringBlockSizeBytes);
						file.read(raw_string_block.data(), stringBlockSizeBytes);
						
						size_t offset = 0;
						while (offset < raw_string_block.size()) {
							std::string name(&raw_string_block[offset]);
							out_global_name_buffer.Add_Name(name);
							offset += name.size() + 1;
						}
					}
					file.close();
				}
			}

			// Unpack current targeted single megachunk geometry file
			std::string fileName = inputFolder + "/megachunk_" + std::to_string(current_megaX) + "_" + std::to_string(current_megaZ) + ".bin";
			if (!std::filesystem::exists(fileName)) return false;

			std::ifstream file(fileName, std::ios::binary);
			if (!file.is_open()) return false;

			char magic[4]; file.read(magic, 4);
			uint32_t fileVersion = 0; file.read(reinterpret_cast<char*>(&fileVersion), 4);

			std::vector<uint64_t> lookupTable(256, 0ULL);
			file.read(reinterpret_cast<char*>(lookupTable.data()), lookupTable.size() * sizeof(uint64_t));
			for (int subX = 0; subX < 16; subX++) {
				for (int subZ = 0; subZ < 16; subZ++) {
					uint8_t lutIndex = ((subZ & 0xF) << 4) | (subX & 0xF);
					uint64_t chunkOffset = lookupTable[lutIndex];
					if (chunkOffset == 0ULL) continue;
					file.seekg(chunkOffset);
					uint32_t nodeCount = 0, wayCount = 0, connCount = 0;
					file.read(reinterpret_cast<char*>(&nodeCount), 4);
					file.read(reinterpret_cast<char*>(&wayCount), 4);
					file.read(reinterpret_cast<char*>(&connCount), 4);
					uint32_t idx1D = (subZ << 4) | subX; // 0..255
					if (nodeCount > 0) {
						flat_nodes_storage[idx1D].resize(nodeCount);
						file.read(reinterpret_cast<char*>(flat_nodes_storage[idx1D].data()), nodeCount * sizeof(Packed_Node));
					}
					if (connCount > 0) {
						flat_connections_storage[idx1D].resize(connCount);
						file.read(reinterpret_cast<char*>(flat_connections_storage[idx1D].data()), connCount * sizeof(uint16_t));
					}
					if (wayCount > 0) {
						flat_ways_storage[idx1D].resize(wayCount);
						file.read(reinterpret_cast<char*>(flat_ways_storage[idx1D].data()), wayCount * sizeof(Packed_Way_Header));
					}
				}
			}
			file.close();
			return true;
		}

		const std::vector<std::vector<Packed_Way_Header>>& GetWaysStorage() const { return flat_ways_storage; }
		const std::vector<std::vector<Packed_Node>>& GetNodeStorage() const { return flat_nodes_storage; }
	};
}