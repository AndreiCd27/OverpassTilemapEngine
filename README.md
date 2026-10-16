
# Overpass Tilemap Engine
### (Pack geospatial data into optimized binary files via Overpass API)
### (Work In Progress)

## Libraries
libcurl - for requesting data from a local docker-hosted instance of Overpass API

json (by nlohmann) - for parsing the output from OverpassAPI

proj - for cartographic projections (For Romania - WGS84 to Stereo70)

## Features (as of yet)
### Spatial Chunking Engine
Organizing roads in a quadtree-like structure, allowing fast streaming of geographical regions in games, simulations etc.
### OverpassQL queries
Using the local endpoint for fast queries at large scale, targeting highways, streets and footways

## Setup

### Download 