#pragma once

#include <cmath>
#include <utility>
#include <string>
#include <proj.h>

namespace geo {

	struct LatLonBBox {
		double lat0, lon0, lat1, lon1;

		std::string to_overpass_string() const {
			return std::to_string(lat0) + "," + std::to_string(lon0) + "," +
				std::to_string(lat1) + "," + std::to_string(lon1);
		}
	};


    struct Stereo70BBox {
        int x0, y0, x1, y1;
    };

    class Stereo70Transformer {
    private:
        PJ_CONTEXT* ctx;
        PJ* proj_wgs_to_stereo;
        PJ* proj_stereo_to_wgs;

    public:
        Stereo70Transformer();

        ~Stereo70Transformer();

        void wgs_to_stereo(double lat, double lon, double& x, double& y);

        void stereo_to_wgs(double x, double y, double& lat, double& lon);

    };

	namespace ro
	{
		inline constexpr int MapSize_m = 819200; // 819.2 km

        inline constexpr int OriginX_m = 500000;
        inline constexpr int OriginY_m = 500000;

		inline constexpr LatLonBBox ro_bounds_wgs80 {
			.lat0 = 43.5, // South
			.lon0 = 20.0, // West
			.lat1 = 48.5, // North
			.lon1 = 30.0  // East
		};

        inline constexpr Stereo70BBox ro_bounds_m{
            .x0 = OriginX_m - MapSize_m / 2,
            .y0 = OriginY_m - MapSize_m / 2,
            .x1 = OriginX_m + MapSize_m / 2,
            .y1 = OriginY_m + MapSize_m / 2
        };

	}
}
