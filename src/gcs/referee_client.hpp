#pragma once

#include "controller_mavlink.hpp"
#include "cpr/api.h"
#include "cpr/cookies.h"
#include "cpr/cprtypes.h"
#include "cpr/response.h"
#include "cpr/timeout.h"
#include "field.hpp"
#include "spdlog/spdlog.h"

#include <chrono>
#include <cpr/cpr.h>
#include <nlohmann/json.hpp>
#include <ctime>

using json = nlohmann::json;

namespace rota {
class referee_client {
    static constexpr int TELEM_INTERVAL = 500;
    // using hss_arr_t = std::vector<std::map<std::string, real_t>>;
public:
    struct time_t {
        // int day;
        int hour;
        int minute;
        int second;
        int millisecond;
    };

    struct qr_coords_t {
        real_t latitude;
        real_t longitude;
    };

    struct hss_t {
        int id;
        real_t latitude;
        real_t longitude;
        real_t radius;
    };

    struct server_time_t {
        std::uint64_t days;
        std::uint64_t hours;
        std::uint64_t minutes;
        std::uint64_t seconds;
        std::uint64_t milliseconds;
    };

    using hss_arr_t = std::vector<hss_t>;

    referee_client(const std::string& referee_server_ip, const std::string& team_name, const std::string& pswd, controller_mavlink& plane_controller) :
        _referee_server_ip(referee_server_ip),
        _team_name(team_name),
        _pswd(pswd),
        _mavlink{plane_controller},
        _team_no(),
        _is_running(true),
        _time_offset(0ull),
        _qr_coords(0.0f, 0.0f),
        _hss_arr(),
        _timeout(1000),
        _session(),
        _thread([this]() { thread_func(); })
        {}

    ~referee_client() {
        _is_running = false;
        _thread.join();
    }

    obstacle_t hss_to_obstacle(const hss_t& hss) {
        obstacle_t obs{};
        obs.position = _mavlink.coords_to_xz(hss.latitude, hss.longitude);
        obs.radius = hss.radius;
        return obs;
    }

    static time_t unix_to_time(std::uint64_t unix_time) {
        std::time_t t = unix_time / 1000000ull;
        auto date = *std::gmtime(&t);

        time_t time;
        // time.day = date.tm_mday;
        time.hour = date.tm_hour;
        time.minute = date.tm_min;
        time.second = date.tm_sec;
        time.millisecond = (unix_time / 1000ull) % 1000ull;

        return time;
    }

    referee_client::time_t get_time() const {
        // in microseconds
        std::uint64_t unix_epoch_time = _mavlink.get_unix_epoch_time() + _time_offset;
        // unix_epoch_time -= 1000000;
        return unix_to_time(unix_epoch_time);
    };

    void login() {
        json data = json::object({
            {"kadi", _team_name},
            {"sifre", _pswd}
        });

        _session.SetUrl(get_uri("/api/giris"));
        _session.SetBody(data.dump());
        _session.SetHeader({{"Content-Type", "application/json"}});
        _session.SetTimeout(_timeout);

        auto r = _session.Post();

        if (r.status_code != 200) {
            spdlog::error("Login failed: {}", r.status_code);
            return;
        }

        _team_no = r.text;
        spdlog::info("Login successful.");
        spdlog::info("Password: {}", _team_no);

        _session.SetUrl("/api/sunucusaati");
        r = _session.Get();
        if (r.status_code != 200) {
            spdlog::error("GET request /api/sunucusaati failed with code: {}", r.status_code);
            return;
        }
        else {
            spdlog::info("Sunucu saati alindi.");
        }

        json data_json = json::parse(r.text);
        server_time_t time = parse_time(data_json);
        update_time(time);
    }

    void send_telemetry() {
        auto data = get_telem_data();

        _session.SetUrl(get_uri("/api/telemetri_gonder"));
        _session.SetBody(data.dump());
        auto r = _session.Post();
        spdlog::info(r.text);

        if (r.status_code != 200) {
            spdlog::error("Post request /api/telemetri_gonder failed with code: {}", r.status_code);
            return;
        }

        data = json::parse(r.text);
        server_time_t time = parse_time(data["sunucusaati"]);
        update_time(time);

        spdlog::info(r.text);
    }

    void get_qr_coords() {
        _session.SetUrl(get_uri("/api/qr_koordinati"));
        auto r = _session.Get();
        if (r.status_code != 200) {
            spdlog::error("Get request /api/qr_koordinati failed with code: {}", r.status_code);
            return;
        }

        json data = json::parse(r.text);
        _qr_coords = {
            data["qrEnlem"],
            data["qrBoylam"]
        };
        spdlog::info("QR Code coordinates received: ({}, {})", _qr_coords.latitude, _qr_coords.longitude);
    }

    hss_arr_t get_hss_coords() {
        _session.SetUrl(get_uri("/api/hss_koordinatlari"));
        auto r = _session.Get();
        if (r.status_code != 200) {
            spdlog::error("Get request /api/qr_koordinati failed with code: {}", r.status_code);
            return {};
        }

        json data = json::parse(r.text);

        server_time_t time = parse_time(data["sunucusaati"]);
        update_time(time);

        const auto arr = data["hss_koordinat_bilgileri"];

        _hss_arr.clear();
        for (const auto& hss_data : arr) {
            hss_t hss{};
            hss.id = hss_data["id"];
            hss.latitude = hss_data["hssEnlem"];
            hss.longitude = hss_data["hssBoylam"];
            hss.radius = hss_data["hssYaricap"];

            _hss_arr.push_back(hss);
        }

        return _hss_arr;
    }

    void send_lock_info(time_t start, time_t end) {
        json data = json::object({
            {
                "kilitlenmeBaslangicZamani", {
                    {"saat", start.hour},
                    {"dakika", start.minute},
                    {"saniye", start.second},
                    {"milisaniye", start.millisecond}
                },
            },
            {
                "kilitlenmeBitisZamani", {
                    {"saat", end.hour},
                    {"dakika", end.minute},
                    {"saniye", end.second},
                    {"milisaniye", end.millisecond}
                }
            },
            {"otonom_kilitlenme", 1}
        });

        _session.SetUrl(get_uri("/api/kilitlenme_bilgisi"));
        _session.SetBody(data.dump());
        auto r = _session.Post();

        if (r.status_code != 200) {
            spdlog::error("Post request /api/kilitlenme_bilgisi failed with code: {}", r.status_code);
            return;
        }
    }

    void send_kamikaze_info(time_t start, time_t end, const std::string& text) {
        json data = json::object({
            {
                "kamikazeBaslangicZamani",
                {
                    {"saat", start.hour},
                    {"dakika", start.minute},
                    {"saniye", start.second},
                    {"milisaniye", start.millisecond}
                },
            },
            {
                "kamikazeBitisZamani",
                {
                    {"saat", end.hour},
                    {"dakika", end.minute},
                    {"saniye", end.second},
                    {"milisaniye", end.millisecond}
                },
            },
            {"qrMetni", text}
        });

        _session.SetUrl(get_uri("/api/kamikaze_bilgisi"));
        _session.SetBody(data.dump());
        auto r = _session.Post();

        if (r.status_code != 200) {
            spdlog::error("Post request /api/kamikaze_bilgisi failed with code: {}", r.status_code);
            return;
        }
    }

private:

    std::string get_uri(const std::string& route) const {
        return "http://" + _referee_server_ip + route;
    }

    std::string _referee_server_ip;
    std::string _team_name;
    std::string _pswd;
    controller_mavlink& _mavlink;
    std::string _team_no;
    std::atomic_bool _is_running;
    std::uint64_t _time_offset;
    qr_coords_t _qr_coords;
    hss_arr_t _hss_arr;
    int _timeout;
    cpr::Session _session;
    std::thread _thread;

    server_time_t parse_time(const json& data) const {
        return {
            data["gun"],
            data["saat"],
            data["dakika"],
            data["saniye"],
            data["milisaniye"]
        };
    };

    void update_time(server_time_t time) {
        tm utc;
        utc.tm_year = 2024 - 1900;
        utc.tm_mon = 8;
        utc.tm_mday = time.days;
        utc.tm_hour = time.hours;
        utc.tm_min = time.minutes;
        utc.tm_sec = time.seconds;
        utc.tm_isdst = 0;

        std::time_t server_time = timegm(&utc) * 1000000ull;
        std::uint64_t gps_time = _mavlink.get_unix_epoch_time();

        _time_offset = server_time - gps_time;
    }

    real_t convert_yaw(real_t yaw) const {
        if (yaw < 0.0f) {
            return 360.0f + yaw;
        }
        else {
            return yaw;
        }
    }

    json get_telem_data() const {
        coords_t coords = _mavlink.get_coords();
        attitude_t attitude = _mavlink.get_attitude();
        velocity_t vel = _mavlink.get_velocity();
        battery_t battery = _mavlink.get_battery();
        referee_client::time_t time = get_time();
        flight_mode_t flight_mode = _mavlink.get_flight_mode();

        json data = json::object({
            {"takim_numarasi", std::stoi(_team_no)},
            {"iha_enlem", _mavlink.get_coords().lat},
            {"iha_boylam", _mavlink.get_coords().lon},
            {"iha_irtifa", static_cast<int64_t>(_mavlink.get_coords().alt)},
            {"iha_dikilme", static_cast<int64_t>(_mavlink.get_attitude().pitch)},
            {"iha_yonelme", static_cast<int64_t>(convert_yaw(_mavlink.get_attitude().yaw))},
            {"iha_yatis", static_cast<int64_t>(_mavlink.get_attitude().roll)},
            {"iha_hiz", static_cast<int64_t>(std::sqrt((vel.north * vel.north) + (vel.east * vel.east)))},
            {"iha_batarya", static_cast<int64_t>(_mavlink.get_battery().remaining_percent)},
            {"iha_otonom", static_cast<int64_t>(
                flight_mode != flight_mode_t::Manual &&
                flight_mode != flight_mode_t::Stabilized &&
                flight_mode != flight_mode_t::Acro &&
                flight_mode != flight_mode_t::Rattitude &&
                flight_mode != flight_mode_t::Posctl &&
                flight_mode != flight_mode_t::Altctl
            )},
            {"iha_kilitlenme", 0},
            {"hedef_merkez_X", 0},
            {"hedef_merkez_Y", 0},
            {"hedef_genislik", 0},
            {"hedef_yukseklik", 0},
            {"gps_saati", {
                {"saat", time.hour},
                {"dakika", time.minute},
                {"saniye", time.second},
                {"milisaniye", time.millisecond}
             }}
        });

        return data;
    }

    void thread_func() {
        while (_is_running) {
            if (_team_no.empty()) {
                spdlog::error("Login required for referee server.");
                std::this_thread::sleep_for(std::chrono::milliseconds(TELEM_INTERVAL));
                continue;
            }

            send_telemetry();
            std::this_thread::sleep_for(std::chrono::milliseconds(TELEM_INTERVAL));
        }
    }
};
}