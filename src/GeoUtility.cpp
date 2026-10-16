
#include "GeoUtility.h"

using namespace geo;

Stereo70Transformer::Stereo70Transformer() {

    ctx = proj_context_create();

    const char* stereo70_def = "+proj=sterea +lat_0=46 +lon_0=25 +k=0.99975 +x_0=500000 +y_0=500000 +ellps=krass +units=m +no_defs";

    // Bidirectional Transform
    // WGS84 is EPSG:4326
    proj_wgs_to_stereo = proj_create_crs_to_crs(ctx, "EPSG:4326", stereo70_def, nullptr);
    proj_stereo_to_wgs = proj_create_crs_to_crs(ctx, stereo70_def, "EPSG:4326", nullptr);
}

Stereo70Transformer::~Stereo70Transformer() {

    if (proj_wgs_to_stereo) proj_destroy(proj_wgs_to_stereo);
    if (proj_stereo_to_wgs) proj_destroy(proj_stereo_to_wgs);
    if (ctx) proj_context_destroy(ctx);
}

void Stereo70Transformer::wgs_to_stereo(double lat, double lon, double& x, double& y) {

    PJ_COORD c_in = proj_coord(lat, lon, 0.0, 0.0);
    PJ_COORD c_out = proj_trans(proj_wgs_to_stereo, PJ_FWD, c_in);
    x = c_out.xy.x;
    y = c_out.xy.y;
}

void Stereo70Transformer::stereo_to_wgs(double x, double y, double& lat, double& lon) {
    PJ_COORD c_in = proj_coord(x, y, 0.0, 0.0);
    PJ_COORD c_out = proj_trans(proj_stereo_to_wgs, PJ_FWD, c_in);
    lat = c_out.lp.lam;
    lon = c_out.lp.phi;
}
