
#include "Request.h"
#include "GeoUtility.h"
#include "OsmParse.h"
#include "OsmPack.h"
#include "ChunkManager.h"


#include <string>
#include <filesystem>

constexpr int divChunk = 16;

static std::string GetQuery(double lat0, double lon0, double lat1, double lon1) {
	
	char bbox_buf[100];
	std::snprintf(bbox_buf, sizeof(bbox_buf), "%.6f,%.6f,%.6f,%.6f",
		lat0, lon0, lat1, lon1);
	std::string bbox(bbox_buf);
	return "[out:json][timeout:90];\n"
		"(way[\"highway\"~\"^(motorway|trunk|primary|secondary|tertiary|residential|pedestrian|footway|path)$\"]("
		+ bbox + "););\n"
		"out body geom;";
}

int main() {

	std::filesystem::create_directories("ChunkData");

	curl_global_init(CURL_GLOBAL_DEFAULT);

	std::string url = "http://localhost:12345/api/interpreter";
	
	CurlWrapper wrapper;

	int xi_global = 9;
	int yi_global = 4;

	ChunkManager chunk_manager(divChunk);
	transformer T;

	osm::OSM_Parser_Storage parser;

	auto init_chk = chunk_manager.MakeChunk_at(xi_global, yi_global, nullptr);
	std::cout << "X0 : " << init_chk.Bounds_m.x0 << ", Y0 : " << init_chk.Bounds_m.y0 << "\n";


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
			std::string query = GetQuery(sub_min_lat, sub_min_lon, sub_max_lat, sub_max_lon);

			// Response from query
			std::string readBuffer = wrapper.PostUrl(url, query);

			if (!readBuffer.empty() && readBuffer[0] == '<') {
				//std::cerr << "Received HTML instead of JSON: " << readBuffer << "\n";
			}

			parser.ParseJSON(readBuffer);

			//std::cout << parser.get_node_counter()<<" unique nodes \n";

			//IntegrityCheck(i, j, chk);

		}
		std::cout << std::to_string(float(i+1) * 100.0f / float(divChunk)) << "% \n";
	}

	std::cout << "Packing OSM data ...\n";

	osm::OSM_Packed_Storage<2> osmpack{ parser, chunk_manager };

	std::cout << "Writing Packed OSM data ...\n";

	osmpack.WriteToDisk("ChunkData", parser.Get__name_buffer());

	std::cout << "Write complete ...\n";

	osm::NameBuffer out_names;
		
	osmpack.ReadFromDisk("ChunkData", out_names);

	std::cout << out_names.size() << " Unique Road Names\n";

	int err_cnt = 0;
	std::vector<osm::Packed_Way_Header> err_obj_catch;

	const auto& ways_store = osmpack.GetWaysStorage();
	std::cout << ways_store.size() << " : Ways Storage Size\n";
	for (int i = 0; i < ways_store.size(); i++) {
		const auto& chunk_ways_store = ways_store[i];
		for (const auto& way : chunk_ways_store) {
			std::cout << int(way.nr_of_nodes) << " : Number of nodes\n";
			std::cout << int(way.name_idx) << " : Road Index\n";
			try {
				const auto& name = out_names.get_name(int(way.name_idx));
				std::cout << name << " : Road Name\n";
			}
			catch (...) {
				err_cnt++;
				err_obj_catch.push_back(way);
			}
		}
	}

	std::cout << "Error count : " << err_cnt << "\n";
	std::cout << "Corrupted objects : \n";
	for (const auto& w : err_obj_catch) {
		std::cout << " * Way from Node " << int(w.first_node_index) << "\n";
		std::cout << " | " << int(w.name_idx) << " : Name Index \n";
	}
	
	//ExportGlobalWorld("ChunkData/world.bin", all_chunks);

	// Clean up global libcurl state
	curl_global_cleanup();

	std::cout << "\nENTER to close..." << std::endl;
	std::cin.get();

	return 0;
}