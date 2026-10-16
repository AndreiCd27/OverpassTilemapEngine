
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

### Docker
To create a clone of the OverpassAPI locally, scoped down to a specific region, you first need to
download a .pbf file from Geofabrik (https://download.geofabrik.de/)

Next, convert the downloaded .pbf file into a .bz2 file and rename as "planet.osm.bz2"

Example:
```
osmium cat romania-latest.osm.pbf -o planet.osm.bz2
```

Then create a directory where the database will be stored and run the following docker command:

```
docker run -d \
  --name your_overpass_instance_name \
  --restart always \
  -p 12345:80 \
  -v "/path/to/directory/db:/db" \
  wiktorn/overpass-api
```

After the command finished, check it was created and the ngnix deamon started, ready to query
```
docker logs -f your_overpass_instance_name
```

### Build process

Simply run the CMakelists.txt file
(Testing on Linux and MacOS for compatibility soon)