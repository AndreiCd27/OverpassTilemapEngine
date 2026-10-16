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
		transformer t;
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
				return;
			}

			for (int l = 0; l < NumLayers; l++) {
				double size_x = Bounds.x1 - Bounds.x0;
				double size_y = Bounds.y1 - Bounds.y0;
				double dx = x - double(Bounds.x0);
				double dy = y - double(Bounds.y0);

				ChunkIndex chk_idx{ .X = static_cast<int>(dx / size_x), .Y = static_cast<int>(dy / size_y) };
				out.chk_idx_array[l] = chk_idx;

				chk_meta = chk.MakeChunk_at(chk_idx.X, chk_idx.Y, &chk_meta);
				Bounds = chk_meta.Bounds_m;
			}
		}
	};

	template <int NumLayers>
	class OSM_Packed_Storage {
		std::vector<std::vector<Packed_Way_Header>> flat_ways_storage;
		std::vector<std::vector<Packed_Node>>       flat_nodes_storage;
		std::vector<std::vector<uint16_t>>          flat_connections_storage;

		inline uint32_t GetFlatIndex(const ChunkIndexArray<NumLayers>& indices) const {
			
			int megaX = indices.chk_idx_array[0].X;
			int megaZ = indices.chk_idx_array[0].Y;
			int subX = indices.chk_idx_array[1].X;
			int subZ = indices.chk_idx_array[1].Y;
			return ((megaX & 0xF) << 12) | ((megaZ & 0xF) << 8) | ((subX & 0xF) << 4) | (subZ & 0xF);
		}

	public:
		OSM_Packed_Storage(const OSM_Parser_Storage& parser_storage, const ChunkManager& chunk_manager) {
			// Pre-alocate memory
			flat_ways_storage.resize(65536);
			flat_nodes_storage.resize(65536);
			flat_connections_storage.resize(65536);

			OSM_Chunk_Layout<NumLayers> layout(chunk_manager);

			const auto& global_nodes = parser_storage.Get__nodes_by_osm_id();
			const auto& global_ways = parser_storage.Get__ways();

			// Key is FlatChunkIndex, value is [OSM_ID -> Local_Index_In_Chunk]
			std::unordered_map<uint32_t, std::unordered_map<uint64_t, uint16_t>> node_global_to_local_idx;

			transformer t_node;

			// Process and put nodes in their subchunk
			for (const auto& [node_id, osm_node] : global_nodes) {
				ChunkIndexArray<NumLayers> indices;
				layout.GetChunksIndices(osm_node, indices);

				if (indices.chk_idx_array[0].X == -1) continue;

				uint32_t idx1D = GetFlatIndex(indices);

				double stereo_x, stereo_y;
				t_node.wgs_to_stereo(osm_node.latitude, osm_node.longitude, stereo_x, stereo_y);

				ChunkMetadata leaf_meta = chunk_manager.GetLeafMetadata(indices);

				double norm_x = (stereo_x - leaf_meta.Bounds_m.x0) / (leaf_meta.Bounds_m.x1 - leaf_meta.Bounds_m.x0);
				double norm_y = (stereo_y - leaf_meta.Bounds_m.y0) / (leaf_meta.Bounds_m.y1 - leaf_meta.Bounds_m.y0);

				Packed_Node packed_node;
				packed_node.x = static_cast<uint16_t>(norm_x * 65535.0);
				packed_node.y = static_cast<uint16_t>(norm_y * 65535.0);
				packed_node.elevation = 0.0f;

				auto& chunk_nodes = flat_nodes_storage[idx1D];
				uint16_t local_idx = static_cast<uint16_t>(chunk_nodes.size());

				chunk_nodes.push_back(packed_node);

				node_global_to_local_idx[idx1D][node_id] = local_idx;
			}

			// Process ways
			for (const auto& osm_way : global_ways) {
				if (osm_way.nodes.empty()) continue;

				// Get target chunk from Node 0
				// [Node 0] <------> [Node 1, Node 2, ...]
				uint32_t target_chunk_idx = 0xFFFFFFFF;
				for (uint64_t node_id : osm_way.nodes) {
					auto it_node = global_nodes.find(node_id);
					if (it_node != global_nodes.end()) {
						ChunkIndexArray<NumLayers> indices;
						layout.GetChunksIndices(it_node->second, indices);
						if (indices.chk_idx_array[0].X != -1) {
							target_chunk_idx = GetFlatIndex(indices);
							break;
						}
					}
				}

				if (target_chunk_idx == 0xFFFFFFFF) continue;

				auto& connection_pool = flat_connections_storage[target_chunk_idx];

				Packed_Way_Header way_header;
				way_header.name_idx = static_cast<uint16_t>(osm_way.unique_name_index);
				way_header.neighbour_chunk_id = 0;
				way_header.type_of_way = osm_way.typeOfRoad;
				way_header.bucket0 = (osm_way.laneCount & 0x0F) | ((osm_way.oneway ? 1 : 0) << 4);
				way_header.bucket1 = 0;
				way_header.bucket2 = 0;
				way_header.first_node_index = static_cast<uint32_t>(connection_pool.size());

				uint16_t valid_nodes_count = 0;
				for (uint64_t global_node_id : osm_way.nodes) {
					// A node is added to a way only if it belongs to the current subchunk
					auto& local_node_map = node_global_to_local_idx[target_chunk_idx];
					auto it_local = local_node_map.find(global_node_id);

					if (it_local != local_node_map.end()) {
						connection_pool.push_back(it_local->second);
						valid_nodes_count++;
					}
				}

				way_header.nr_of_nodes = valid_nodes_count;

				if (valid_nodes_count > 0) {
					flat_ways_storage[target_chunk_idx].push_back(way_header);
				}
			}
		}

		void WriteToDisk(const std::string& outputFolder, const NameBuffer& name_buffer) {
			std::filesystem::create_directories(outputFolder);

			// Save global NameBuffer
			{
				std::string globalNamesFile = outputFolder + "/global_names.bin";
				std::ofstream file(globalNamesFile, std::ios::binary);
				if (file.is_open()) {
					// Magic Header for name file
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

			// Save Megachunks

			for (int megaX = 0; megaX < 16; megaX++) {
				for (int megaZ = 0; megaZ < 16; megaZ++) {

					bool hasData = false;
					for (int subX = 0; subX < 16; subX++) {
						for (int subZ = 0; subZ < 16; subZ++) {
							uint32_t idx1D = ((megaX & 0xF) << 12) | ((megaZ & 0xF) << 8) | ((subX & 0xF) << 4) | (subZ & 0xF);
							if (!flat_nodes_storage.at(idx1D).empty() || !flat_ways_storage.at(idx1D).empty()) {
								hasData = true;
								break;
							}
						}
						if (hasData) break;
					}

					if (!hasData) continue;

					std::string fileName = outputFolder + "/megachunk_" + std::to_string(megaX) + "_" + std::to_string(megaZ) + ".bin";
					std::ofstream file(fileName, std::ios::binary);
					if (!file.is_open()) continue;

					// Magic Header for geometry data
					char magic[4] = { 'R', 'O', 'M', 'S' };
					file.write(magic, 4);
					uint32_t fileVersion = 1;
					file.write(reinterpret_cast<char*>(&fileVersion), 4);

					// Local Lookup Table
					std::vector<uint64_t> lookupTable(256, 0ULL);
					uint64_t lutPosition = file.tellp();
					file.write(reinterpret_cast<char*>(lookupTable.data()), lookupTable.size() * sizeof(uint64_t));

					// Serialize subchunks
					for (int subX = 0; subX < 16; subX++) {
						for (int subZ = 0; subZ < 16; subZ++) {

							uint32_t idx1D = ((megaX & 0xF) << 12) | ((megaZ & 0xF) << 8) | ((subX & 0xF) << 4) | (subZ & 0xF);

							const auto& nodes = flat_nodes_storage.at(idx1D);
							const auto& ways = flat_ways_storage.at(idx1D);
							const auto& connections = flat_connections_storage.at(idx1D);

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
			}
		}

		bool ReadFromDisk(const std::string& inputFolder, NameBuffer& out_global_name_buffer) {
			// Clear vectors and reallocate
			flat_ways_storage.assign(65536, std::vector<Packed_Way_Header>());
			flat_nodes_storage.assign(65536, std::vector<Packed_Node>());
			flat_connections_storage.assign(65536, std::vector<uint16_t>());

			// Read NameBuffer file
			std::string globalNamesFile = inputFolder + "/global_names.bin";
			if (!std::filesystem::exists(globalNamesFile)) {
				std::cerr << "[Error] File Not Found: " << globalNamesFile << std::endl;
				return false;
			}

			{
				std::ifstream fileNames(globalNamesFile, std::ios::binary);
				if (!fileNames.is_open()) {
					std::cerr << "[Error] Cannot open file" << std::endl;
					return false;
				}

				char magicNames[4];
				fileNames.read(magicNames, 4);
				if (magicNames[0] != 'R' || magicNames[1] != 'O' || magicNames[2] != 'M' || magicNames[3] != 'N') { // 'ROMN'
					std::cerr << "[Error] Invalid file format" << std::endl;
					return false;
				}

				uint32_t versionNames = 0;
				fileNames.read(reinterpret_cast<char*>(&versionNames), 4);
				if (versionNames != 1) {
					std::cerr << "[Error] Unsupported version of file" << std::endl;
					return false;
				}

				uint16_t totalGlobalNames = 0;
				uint32_t stringBlockSizeBytes = 0;
				fileNames.read(reinterpret_cast<char*>(&totalGlobalNames), 2);
				fileNames.read(reinterpret_cast<char*>(&stringBlockSizeBytes), 4);

				std::vector<char> raw_string_block(stringBlockSizeBytes);
				if (stringBlockSizeBytes > 0) {
					fileNames.read(raw_string_block.data(), stringBlockSizeBytes);
				}

				out_global_name_buffer.setup(raw_string_block, totalGlobalNames);
				fileNames.close();
			}

			// Load Megachunk file
			bool loaded_at_least_one = false;

			for (int megaX = 0; megaX < 16; megaX++) {
				for (int megaZ = 0; megaZ < 16; megaZ++) {

					std::string fileName = inputFolder + "/megachunk_" + std::to_string(megaX) + "_" + std::to_string(megaZ) + ".bin";

					if (!std::filesystem::exists(fileName)) {
						continue;
					}

					std::ifstream file(fileName, std::ios::binary);
					if (!file.is_open()) {
						std::cerr << "[Warning] File Not Found: " << fileName << std::endl;
						continue;
					}

					// A. Validare Magic Header & Versiune Geometrie
					char magic[4];
					file.read(magic, 4);
					if (magic[0] != 'R' || magic[1] != 'O' || magic[2] != 'M' || magic[3] != 'S') { // 'ROMS'
						std::cerr << "[Error] File " << fileName << " has invalid format" << std::endl;
						return false;
					}

					uint32_t fileVersion = 0;
					file.read(reinterpret_cast<char*>(&fileVersion), 4);
					if (fileVersion != 1) {
						std::cerr << "[Error] Version unsupported for file : " << fileName << std::endl;
						return false;
					}

					// Read Lookup Table
					std::vector<uint64_t> lookupTable(256, 0ULL);
					file.read(reinterpret_cast<char*>(lookupTable.data()), lookupTable.size() * sizeof(uint64_t));

					// Deserialize chunks
					for (int subX = 0; subX < 16; subX++) {
						for (int subZ = 0; subZ < 16; subZ++) {

							uint8_t lutIndex = ((subZ & 0xF) << 4) | (subX & 0xF);
							uint64_t chunkOffset = lookupTable[lutIndex];

							// Subchunk is empty
							if (chunkOffset == 0ULL) {
								continue;
							}

							file.seekg(chunkOffset);

							uint32_t nodeCount = 0;
							uint32_t wayCount = 0;
							uint32_t connCount = 0;

							file.read(reinterpret_cast<char*>(&nodeCount), 4);
							file.read(reinterpret_cast<char*>(&wayCount), 4);
							file.read(reinterpret_cast<char*>(&connCount), 4);

							uint32_t idx1D = ((megaX & 0xF) << 12) | ((megaZ & 0xF) << 8) | ((subX & 0xF) << 4) | (subZ & 0xF);

							// Read Nodes
							if (nodeCount > 0) {
								auto& nodes = flat_nodes_storage.at(idx1D);
								nodes.resize(nodeCount);
								file.read(reinterpret_cast<char*>(nodes.data()), nodeCount * sizeof(Packed_Node));
							}

							// Read Connections
							if (connCount > 0) {
								auto& connections = flat_connections_storage.at(idx1D);
								connections.resize(connCount);
								file.read(reinterpret_cast<char*>(connections.data()), connCount * sizeof(uint16_t));
							}

							// Read Ways
							if (wayCount > 0) {
								auto& ways = flat_ways_storage.at(idx1D);
								ways.resize(wayCount);
								file.read(reinterpret_cast<char*>(ways.data()), wayCount * sizeof(Packed_Way_Header));
							}
						}
					}

					file.close();
					loaded_at_least_one = true;
				}
			}

			return loaded_at_least_one;
		}

		/*
		std::vector<std::vector<Packed_Way_Header>> flat_ways_storage;
		std::vector<std::vector<Packed_Node>>       flat_nodes_storage;
		std::vector<std::vector<uint16_t>>          flat_connections_storage;
		*/

		const std::vector<std::vector<Packed_Way_Header>>& GetWaysStorage() const {
			return flat_ways_storage;
		}
		const std::vector<std::vector<Packed_Node>>& GetNodeStorage() const {
			return flat_nodes_storage;
		}
		const std::vector<std::vector<uint16_t>>& GetConnectionsStorage() const {
			return flat_connections_storage;
		}
	};
}