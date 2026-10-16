#pragma once

#include <nlohmann/json.hpp>
#include <vector>
#include <unordered_map>
#include <iostream>
#include <cmath>

#include "OsmTypes.h"
#include "GeoUtility.h"

namespace osm {

    class NameBuffer {
        uint32_t name_index = 0;
        int totalByteCount = 0;
        std::vector<std::string> names;
        std::unordered_map<std::string, uint32_t> names_umap;
    public:
        void setup(const std::vector<char>& raw_string_block, uint16_t totalGlobalNames) {
            
            names.clear();
            names_umap.clear();
            name_index = 0;
            totalByteCount = 0;

            if (raw_string_block.empty() || totalGlobalNames == 0) {
                return;
            }

            names.reserve(totalGlobalNames);
            names_umap.reserve(totalGlobalNames);

            size_t current_pos = 0;
            while (current_pos < raw_string_block.size() && names.size() < totalGlobalNames) {

                const char* current_str_ptr = raw_string_block.data() + current_pos;

                std::string extracted_name(current_str_ptr);

                names.push_back(extracted_name);
                names_umap[extracted_name] = name_index;

                uint32_t bytes_consumed = static_cast<uint32_t>(extracted_name.size() + 1);
                totalByteCount += bytes_consumed;
                name_index++;

                current_pos += bytes_consumed;
            }

            if (names.size() != totalGlobalNames) {
                std::cout << "======= WARNING :: NAME BUFFER SETUP MISMATCH (Expected "
                    << totalGlobalNames << ", Parsed " << names.size() << ") =======\n";
            }
        }
        uint32_t Add_Name(const std::string& name) {

            if (names_umap.find(name) != names_umap.end()) {
                return names_umap[name];
            }

            if (name_index > 65535) {
                std::cout << "======= ERROR :: ILLEGAL NAME INDEX DETECTED =========================\n";
                return name_index - 1;
            }

            totalByteCount += name.size() + 1;
            names.push_back(name);
            names_umap[name] = name_index;
            name_index++;

            return name_index - 1;
        }
        std::string get_name(int name_idx) const {
            if (name_idx >= names.size() || name_idx < 0) {
                std::cout << "ERROR :: INVALID NAME INDEX\n";
                return "";
            }
            return names[name_idx];
        }
        int size() const { return names.size(); }
    };

	class OSM_Parser_Storage {

        uint16_t internal_node_counter = 0;
        std::unordered_map<uint64_t, uint16_t> node_index_by_osm_id;
        std::unordered_map<uint64_t, OSM_Node> nodes_by_osm_id;
		std::vector<OSM_Way> ways;
        NameBuffer name_buffer;

	public:
		void Clear() {
            nodes_by_osm_id.clear();
			ways.clear();
		}
		void ParseJSON(const std::string& rawJsonString);
        uint16_t get_node_counter() const { return internal_node_counter; }
        const std::unordered_map<uint64_t, uint16_t>& Get__node_index_by_osm_id() const {
            return node_index_by_osm_id;
        };
        const std::unordered_map<uint64_t, OSM_Node>& Get__nodes_by_osm_id() const {
            return nodes_by_osm_id;
        };
        const std::vector<OSM_Way>& Get__ways() const {
            return ways;
        };
        const NameBuffer& Get__name_buffer() const {
            return name_buffer;
        };
	};

}