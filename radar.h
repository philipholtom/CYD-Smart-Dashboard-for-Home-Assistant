#pragma once
// Fetches adsb.lol and summarises the nearest aircraft as plain text for the
// radar_page labels in cyd.yaml. (Originally drew a live radar scope on an
// LVGL canvas; switched to text-only - the canvas was both the biggest heap
// hog on this non-PSRAM board and, at 140px, too small to read comfortably.)

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include <algorithm>
#include "esphome/components/json/json_util.h"
#include "esphome/core/log.h"

namespace radar {

constexpr double DEG2RAD = 0.017453292519943295;
constexpr double RAD2DEG = 57.29577951308232;
constexpr double EARTH_RADIUS_NM = 3440.065;

inline const char *compass(double deg) {
  static const char *pts[16] = {"N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
                                "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"};
  int i = ((int) lround(deg / 22.5)) % 16;
  if (i < 0) i += 16;
  return pts[i];
}

inline std::string trimmed(const char *s) {
  std::string v(s ? s : "");
  size_t a = v.find_first_not_of(' ');
  if (a == std::string::npos) return "";
  size_t b = v.find_last_not_of(' ');
  return v.substr(a, b - a + 1);
}

struct PlaneInfo {
  double dist_nm = 0, bearing = 0, gs = 0;
  int alt_ft = 0;
  bool on_ground = false;
  std::string callsign, reg, type;
};

struct RadarSummary {
  int count = 0;
  char status[40] = "--";
  char callsign[16] = "--";
  char line1[32] = "";
  char line2[32] = "";
  char sub[32] = "";
  char next1[40] = "";
  char next2[40] = "";
};

/// Parses the adsb.lol response and fills in the label text for the nearest
/// couple of aircraft. `home_lat/home_lon` - this board has no GPS.
inline RadarSummary summarize(double home_lat, double home_lon, double range_nm,
                              const std::string &body, bool http_ok) {
  RadarSummary out;

  if (!http_ok || body.empty()) {
    ESP_LOGW("radar", "no data (http_ok=%d, body_len=%u)", http_ok, (unsigned) body.size());
    snprintf(out.status, sizeof(out.status), "signal lost");
    return out;
  }

  const double lat1 = home_lat * DEG2RAD, lon1 = home_lon * DEG2RAD;
  std::vector<PlaneInfo> nearby;

  bool parsed = json::parse_json(body, [&](JsonObject root) -> bool {
    JsonArray ac = root["ac"];
    if (ac.isNull()) ac = root["aircraft"];
    if (ac.isNull()) return false;

    for (JsonObject plane : ac) {
      double lat2 = plane["lat"] | 0.0, lon2 = plane["lon"] | 0.0;
      if (lat2 == 0.0 && lon2 == 0.0) continue;

      const double lat2r = lat2 * DEG2RAD, lon2r = lon2 * DEG2RAD;
      const double dlat = lat2r - lat1, dlon = lon2r - lon1;
      const double a = sin(dlat / 2) * sin(dlat / 2) +
                        cos(lat1) * cos(lat2r) * sin(dlon / 2) * sin(dlon / 2);
      const double dist_nm = 2.0 * EARTH_RADIUS_NM * asin(fmin(1.0, sqrt(a)));
      if (dist_nm > range_nm) continue;

      // Skip parked/taxiing aircraft and ground vehicles - "nearest flight"
      // means nearest thing actually flying, not whatever's closest on the ramp.
      bool on_ground = plane["alt_baro"].is<const char *>();
      if (on_ground) continue;

      const double by = sin(dlon) * cos(lat2r);
      const double bx = cos(lat1) * sin(lat2r) - sin(lat1) * cos(lat2r) * cos(dlon);

      PlaneInfo p;
      p.dist_nm = dist_nm;
      p.bearing = fmod(atan2(by, bx) * RAD2DEG + 360.0, 360.0);
      p.on_ground = false;
      p.alt_ft = (int) (plane["alt_baro"] | 0);
      p.gs = plane["gs"] | 0.0;
      p.callsign = trimmed(plane["flight"] | "");
      p.reg = plane["r"] | "";
      p.type = plane["t"] | "";
      nearby.push_back(p);
    }
    return true;
  });

  out.count = (int) nearby.size();

  if (!parsed) {
    ESP_LOGW("radar", "JSON parse failed (body_len=%u)", (unsigned) body.size());
    snprintf(out.status, sizeof(out.status), "signal lost");
    return out;
  }

  std::sort(nearby.begin(), nearby.end(),
            [](const PlaneInfo &a, const PlaneInfo &b) { return a.dist_nm < b.dist_nm; });

  snprintf(out.status, sizeof(out.status), "%d aircraft within %.0fnm", out.count, range_nm);
  ESP_LOGD("radar", "%d aircraft in range", out.count);

  auto name_of = [](const PlaneInfo &p) -> std::string {
    if (!p.callsign.empty()) return p.callsign;
    if (!p.reg.empty()) return p.reg;
    return "UNKNOWN";
  };

  if (!nearby.empty()) {
    const auto &n = nearby[0];
    snprintf(out.callsign, sizeof(out.callsign), "%s", name_of(n).c_str());

    if (n.on_ground)
      snprintf(out.line1, sizeof(out.line1), "On the ground");
    else if (n.alt_ft >= 18000)
      snprintf(out.line1, sizeof(out.line1), "FL%d   %.0f kt", n.alt_ft / 100, n.gs);
    else
      snprintf(out.line1, sizeof(out.line1), "%d ft   %.0f kt", n.alt_ft, n.gs);

    // No parens or degree sign - not reliably in roboto_16's compiled glyph
    // set (showed as a blank box on-device), so spell it out instead.
    snprintf(out.line2, sizeof(out.line2), "%.1f nm   %s  %03.0f deg",
             n.dist_nm, compass(n.bearing), n.bearing);

    if (!n.reg.empty() || !n.type.empty())
      snprintf(out.sub, sizeof(out.sub), "%s   %s", n.reg.c_str(), n.type.c_str());
  }
  for (size_t i = 1; i < nearby.size() && i <= 2; i++) {
    const auto &n = nearby[i];
    char *dst = (i == 1) ? out.next1 : out.next2;
    size_t sz = (i == 1) ? sizeof(out.next1) : sizeof(out.next2);
    snprintf(dst, sz, "%-9s %5d ft   %.1f nm", name_of(n).c_str(), n.alt_ft, n.dist_nm);
  }

  return out;
}

}  // namespace radar
