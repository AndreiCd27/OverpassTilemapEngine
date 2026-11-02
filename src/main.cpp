
#include "Request.h"
#include "GeoUtility.h"
#include "OsmParse.h"
#include "OsmPack.h"
#include "ChunkManager.h"
#include "LZ4_Compression.h"

#include <string>
#include <filesystem>
#include <chrono>
#include <thread>

#include <fstream>

std::ofstream fout("log.txt");

constexpr int divChunk = 16;

static std::string GetQuery(double lat0, double lon0, double lat1, double lon1) {
	
	char bbox_buf[100];
	std::snprintf(bbox_buf, sizeof(bbox_buf), "%.6f,%.6f,%.6f,%.6f",
		lat0, lon0, lat1, lon1);
	std::string bbox(bbox_buf);
	return "[out:json][timeout:90];\n"
		"(\n"
		"  way[\"highway\"~\"^(motorway|trunk|primary|secondary|tertiary|residential|pedestrian|footway|path|construction)$\"](" + bbox + ");\n"
		"  way[\"railway\"~\"^(rail|tram|track|construction|disused)$\"](" + bbox + ");\n"
		");\n"
		"out body geom;";
}

static std::string GetHeightmapQuery(double lat0, double lon0, double lat1, double lon1) {
	// Here we return the query for OpenTopoData API
	std::string locations = "";
	const int res = 16;
	for (int x = 0; x < res; x++) {
		for (int y = 0; y < res; y++) {
			float fx = float(x) / float(res-1);
			float fy = float(y) / float(res-1);
			float flat = lat0 * (1 - fy) + lat1 * fy;
			float flon = lon0 * (1 - fx) + lon1 * fx;
			locations += std::format("{:.6f},{:.6f}", flat, flon);
			if (!(x == res - 1 && y == res - 1)) {
				locations += "|";
			}
		}
	}
	using json = nlohmann::json;
	json request_json;
	request_json["locations"] = locations;

	return request_json.dump();
}

int main() {

	std::filesystem::create_directories("ChunkData");

	curl_global_init(CURL_GLOBAL_DEFAULT);

	std::string url = "http://localhost:12345/api/interpreter";
	std::string url_elevation = "http://localhost:5000/v1/ro_srtm30m?locations=";
	
	CurlWrapper wrapper;

	int xi_global = 9;
	int yi_global = 4;

	ChunkManager chunk_manager(divChunk);
	transformer T;

	osm::OSM_Parser_Storage parser;

	Heightmap_Storage chunk_heightmaps(64, 16);

	auto init_chk = chunk_manager.MakeChunk_at(xi_global, yi_global, nullptr);
	int _x0 = init_chk.Bounds_m.x0;
	int _y0 = init_chk.Bounds_m.y0;
	int _x1 = init_chk.Bounds_m.x1;
	int _y1 = init_chk.Bounds_m.y1;
	std::cout << "X0 : " << _x0 << ", Y0 : " << _y0 << "\n";
	std::cout << "X1 : " << _x1 << ", Y1 : " << _y1 << "\n";
	std::cout << "Xsize : " << _x1 - _x0 << ", Ysize : " << _y1 - _y0 << "\n";

	double mega_min_lat, mega_min_lon, mega_max_lat, mega_max_lon;
	T.stereo_to_wgs(init_chk.Bounds_m.x0, init_chk.Bounds_m.y0, mega_min_lat, mega_min_lon);
	T.stereo_to_wgs(init_chk.Bounds_m.x1, init_chk.Bounds_m.y1, mega_max_lat, mega_max_lon);

	std::cout << mega_min_lat << " " << mega_min_lon << " "
		<< mega_max_lat << " " << mega_max_lon << "\n";

	for (int i = 0; i < divChunk; i++) {
		for (int j = 0; j < divChunk; j++) {

			// Local chunk of defined region
			auto chunk_meta = chunk_manager.MakeChunk_at(i, j, &init_chk);
			auto bounds = chunk_meta.Bounds_m;

			double sub_min_lat, sub_min_lon, sub_max_lat, sub_max_lon;
			T.stereo_to_wgs(bounds.x0, bounds.y0, sub_min_lat, sub_min_lon);
			T.stereo_to_wgs(bounds.x1, bounds.y1, sub_max_lat, sub_max_lon);


			// Query smaller area to reduce response times
			const std::string& query = GetQuery(sub_min_lat, sub_min_lon, sub_max_lat, sub_max_lon);

			// Heightmap query
			const std::string& heightmap_query = 
				GetHeightmapQuery(sub_min_lat, sub_min_lon, sub_max_lat, sub_max_lon);

			// Response from query
			std::string readBuffer = wrapper.PostUrl(url, query);

			if (!readBuffer.empty() && readBuffer[0] == '<') {
				std::cerr << "[VECTOR_BUFFER] Received HTML instead of JSON \n";
				// try again after 50 ms
				std::this_thread::sleep_for(std::chrono::milliseconds(50));
				readBuffer = wrapper.PostUrl(url, query);
			}

			parser.ParseJSON(readBuffer);

			// Response from elevation query
			std::string elv_readBuffer = wrapper.PostUrl(url_elevation, heightmap_query);

			if (!elv_readBuffer.empty() && elv_readBuffer[0] == '<') {
				std::cerr << "[ELEVATION_BUFFER] Received HTML instead of JSON \n";
			}

			chunk_heightmaps.ComputeHeightmapForChunk(elv_readBuffer, i, j);

			//std::cout << parser.get_node_counter()<<" unique nodes \n";

			//IntegrityCheck(i, j, chk);

		}
		std::cout << std::to_string(float(i+1) * 100.0f / float(divChunk)) << "% \n";
	}

	std::cout << "Packing OSM data ...\n";

	osm::OSM_Packed_Storage<2> osmpack{ parser, chunk_manager, xi_global, yi_global };

	std::cout << "Writing Packed OSM data ...\n";

	osmpack.WriteToDisk("ChunkData", parser.Get__name_buffer(), chunk_heightmaps);

	std::cout << "Write complete ...\n";

	osm::NameBuffer out_names;
		
	osmpack.ReadFromDisk("ChunkData", out_names, chunk_heightmaps);

	std::cout << out_names.size() << " Unique Road Names\n";

	int err_cnt = 0;
	std::vector<osm::Packed_Way_Header> err_obj_catch;
	const auto& ways_store = osmpack.GetWaysStorage();
	fout << ways_store.size() << " : Ways Storage Size\n"; // Will print 256

	for (int i = 0; i < ways_store.size(); i++) {
		const auto& chunk_ways_store = ways_store[i];

		if (chunk_ways_store.size() > 0) {
			// Extract indices purely within the local 256 matrix limits
			int subX = i & 0xF;
			int subZ = (i >> 4) & 0xF;
			
			fout << "WAYCHK: [Mega " << xi_global << "," << yi_global << "] "
				<< "-> [Sub " << subX << "," << subZ << "] "
				<< "(Local Idx: " << i << ") | "
				<< chunk_ways_store.size() << " ways\n";

			for (const auto& way : chunk_ways_store) {
				try {
					const auto& name = out_names.get_name(int(way.name_idx));
				}
				catch (...) {
					err_cnt++;
					err_obj_catch.push_back(way);
				}
			}
		}
	}

	const auto& out_nodes = osmpack.GetNodeStorage();
	for (int i = 0; i < out_nodes.size(); i++) {
		if (out_nodes[i].size() > 0) {
			int megaX = (i >> 12) & 0xF;
			int megaZ = (i >> 8) & 0xF;
			int subX = (i >> 4) & 0xF;
			int subZ = i & 0xF;
			int localSubIdx = (subZ << 4) | subX;

			fout << "NDCHK: [Mega " << megaX << "," << megaZ << "] "
				<< "-> [Sub " << subX << "," << subZ << "] "
				<< "(Local Idx: " << localSubIdx << ") | "
				<< out_nodes[i].size() << " nodes\n";
		}
	}

	std::cout << "Error count : " << err_cnt << "\n";
	std::cout << "Corrupted objects : \n";
	for (const auto& w : err_obj_catch) {
		std::cout << " * Way from Node " << int(w.first_node_index) << "\n";
		std::cout << " | " << int(w.name_idx) << " : Name Index \n";
	}
	
	try {
		// Compress vector data binary using LZ4
		std::string bin_file_name = std::format("ChunkData/megachunk_{}_{}.bin", xi_global, yi_global);
		std::vector<char> compressedData = CompressFileLZ4(bin_file_name);

		std::string lz4_file_name = std::format("ChunkData/megachunk_{}_{}.lz4", xi_global, yi_global);
		std::ofstream outFile(lz4_file_name, std::ios::binary);
		outFile.write(compressedData.data(), compressedData.size());
		outFile.close();

		std::cout << "Vector data file compressed successfully via LZ4!\n";

		// Compress name buffer binary using LZ4
		std::vector<char> compressedData_names = CompressFileLZ4("ChunkData/global_names.bin");

		std::ofstream outFile_names("ChunkData/global_names.lz4", std::ios::binary);
		outFile_names.write(compressedData_names.data(), compressedData_names.size());
		outFile_names.close();

		std::cout << "Name buffer file compressed successfully via LZ4!\n";
	}
	catch (const std::exception& e) {
		std::cerr << "Error: " << e.what() << "\n";
	}

	//ExportGlobalWorld("ChunkData/world.bin", all_chunks);

	// Clean up global libcurl state
	curl_global_cleanup();

	fout.close();

	std::cout << "\nENTER to close..." << std::endl;
	std::cin.get();

	return 0;
}