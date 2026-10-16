
#include "OsmParse.h"
#include <algorithm>

using namespace osm;

void OSM_Parser_Storage::ParseJSON(const std::string & rawJsonString) {

    using json = nlohmann::json;

    if (rawJsonString.empty()) {
        std::cerr << "Error, empty JSON string\n";
    }

    try {
        json parsedData = json::parse(rawJsonString);

        if (parsedData.contains("elements") && parsedData["elements"].is_array()) {

            const auto& elements = parsedData["elements"];

            //std::cout << "Identified " << elements.size() << " roads\n";

            for (const auto& element : elements) {
                if (element.value("type", "") == "way") {

                    uint64_t id = element.value("id", 0ULL);

                    std::string streetName = "";

                    if (element.contains("tags") && element["tags"].contains("name")) {
                        streetName = element["tags"]["name"];
                    }

                    uint8_t typeOfRoadCode = 0; // 0 = Drum auto implicit (residential, etc.)

                    if (element.contains("tags")) {
                        const auto& tags = element["tags"];
                        if (tags.contains("highway")) {
                            std::string hwType = tags["highway"].get<std::string>();

                            if (hwType == "motorway" || hwType == "trunk") {
                                typeOfRoadCode = 1; // Autostrăzi / Drumuri expres
                            }
                            else if (hwType == "primary" || hwType == "secondary" || hwType == "tertiary") {
                                typeOfRoadCode = 2; // Drumuri principale / Bulevarde
                            }
                            else if (hwType == "pedestrian") {
                                typeOfRoadCode = 3; // Zonă pietonală urbană
                            }
                            else if (hwType == "footway" || hwType == "path") {
                                typeOfRoadCode = 4; // Alee pietonală / Potecă parcuri
                            }
                        }
                    }

                    // Default Value (1 lane both directions)
                    uint8_t laneCount = 2;

                    if (element.contains("tags")) {
                        const auto& tags = element["tags"];

                        if (tags.contains("lanes") && !tags["lanes"].is_null()) {
                            std::string lanesStr = tags["lanes"].get<std::string>();

                            try {
                                int parsedLanes = std::stoi(lanesStr);
                                laneCount = static_cast<uint8_t>((std::max)(1, (std::min)(15, parsedLanes)));
                            }
                            catch (...) {
                                laneCount = 2;
                            }
                        }
                    }

                    bool isOneway = false;

                    if (element.contains("tags")) {
                        const auto& tags = element["tags"];

                        if (tags.contains("oneway") && !tags["oneway"].is_null()) {
                            std::string onewayStr = tags["oneway"].get<std::string>();
                            if (onewayStr == "yes" || onewayStr == "1") {
                                isOneway = true;
                            }
                            else if (onewayStr == "-1") {
                                isOneway = true;
                                //std::cout << "WARNING :: ONE_WAY_STREET IS -1\n";
                            }
                        }
                    }

                    // Extract geometry (list of nodes)
                    if (element.contains("geometry") && element["geometry"].is_array()) {
                        OSM_Way way;
                        way.laneCount = laneCount;
                        way.oneway = isOneway;
                        way.typeOfRoad = typeOfRoadCode;
                        way.unique_name_index = name_buffer.Add_Name(streetName);

                        if (element.contains("nodes") && element.contains("geometry") &&
                            element["nodes"].size() == element["geometry"].size()) {

                            for (size_t i = 0; i < element["geometry"].size(); i++) {
                                const auto& geo_node = element["geometry"][i];

                                uint64_t osm_id = element["nodes"][i].get<uint64_t>();

                                double Lat = geo_node.value("lat", 0.0);
                                double Lon = geo_node.value("lon", 0.0);

                                OSM_Node osm_node{ .osm_id = osm_id, .longitude = Lon, .latitude = Lat };
                                nodes_by_osm_id[osm_id] = osm_node;

                                if (node_index_by_osm_id.find(osm_id) == node_index_by_osm_id.end()) {
                                    internal_node_counter++;
                                    node_index_by_osm_id[osm_id] = internal_node_counter;
                                }

                                way.nodes.push_back(osm_id);
                            }
                        }

                        if (element.contains("tags") && element["tags"].contains("oneway") &&
                            element["tags"]["oneway"].get<std::string>() == "-1") {
                            std::reverse(way.nodes.begin(), way.nodes.end());
                        }

                        ways.push_back(way);
                    }
                }
            }
        }
        else {
            std::cerr << "Invalid OSM data\n";
        }
    }
    catch (const json::parse_error& e) {
        std::cerr << "JSON Parse error: " << e.what() << "\n";
    }
    catch (const std::exception& e) {
        std::cerr << "JSON Exception: " << e.what() << "\n";
    }
}