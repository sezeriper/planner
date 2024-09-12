#pragma once

#include "controller_mavlink.hpp"

#include <cpr/cpr.h>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace rota {
class referee_client {
    static constexpr int TELEM_INTERVAL = 900;
public:
    referee_client(const std::string& referee_server_ip, controller_mavlink& plane_controller) : _referee_server_ip(referee_server_ip), _mavlink{plane_controller}, _thread([this]() { thread_func(); }) {}

    ~referee_client() {
        _is_running = false;
        _thread.join();
    }

    void login() {
        json my_json = json::object({
            {"kadi", "takimadi"},
            {"sifre", "takimsifresi"},
        });

        cpr::Response r = cpr::Post(
            cpr::Url{"http://" + _referee_server_ip + "/api/giris"},
            cpr::Header{{"Content-Type", "application/json"}},
            cpr::Body{my_json.dump()}
        );
        
        if (r.status_code != 200) {
            spdlog::error("Login failed: {}", r.status_code);
            return;
        }

        _team_no = r.text;
        spdlog::info("Login successful.");
        spdlog::info("Password: {}", _team_no);
    }

    void send_telemetry() const {
        auto telem_data = get_telem_data();

        json data = json::object({
            {"takim_numarasi", _team_no},
            {"iha_enlem", ""},
            {"iha_boylam", ""},
            {"iha_irtifa", ""},
            {"iha_dikilme", ""},
            {"iha_yonelme", ""},
            {"iha_yatis", ""},
            {"iha_hiz", ""},
            {"iha_batarya", ""},
            {"iha_otonom", ""},
            {"iha_kilitlenme", ""},
            {"hedef_merkez_X", ""},
            {"hedef_merkez_Y", ""},
            {"hedef_genislik", ""},
            {"hedef_yukseklik", ""},
            {"gps_saati", }
        });

        cpr::Response r = cpr::Post(
            cpr::Url{"http://" + _referee_server_ip + "/api/telemetri_gonder"},
            cpr::Header{{"Content-Type", "application/json"}},
            cpr::Body{data.dump()}
        );
    }

private:
    struct time_t {
        int hour;
        int minute;
        int second;
        int millisecond;
    };


    time_t get_time() const {
        // in microseconds
        std::uint64_t unix_epoch_time = _mavlink.get_unix_epoch_time();

        time_t time;
        time.hour = (unix_epoch_time / 3600000000ull) % 24ull;
        time.minute = (unix_epoch_time / 60000000ull) % 60ull;
        time.second = (unix_epoch_time / 1000000ull) % 60ull;
        time.millisecond = (unix_epoch_time / 1000ull) % 1000ull;

        return time;
    };

    std::string _referee_server_ip;
    controller_mavlink& _mavlink;
    std::thread _thread;
    std::string _team_no;
    std::atomic_bool _is_running{true};

    json get_telem_data() const {
        time_t time = get_time();

        json data;
        data["iha_enlem"] = _mavlink.get_coords().lat;
        data["iha_boylam"] = _mavlink.get_coords().lon;
        data["iha_irtifa"] = _mavlink.get_coords().alt;
        data["iha_dikilme"] = _mavlink.get_attitude().pitch;
        data["iha_yonelme"] = _mavlink.get_attitude().yaw;
        data["iha_yatis"] = _mavlink.get_attitude().roll;
        data["iha_hiz"] = _mavlink.get_metrics().airspeed;
        data["iha_batarya"] = _mavlink.get_battery().remaining_percent;
        data["iha_otonom"] = 0;
        data["iha_kilitlenme"] = 0;
        data["hedef_merkez_X"] = 0;
        data["hedef_merkez_Y"] = 0;
        data["hedef_genislik"] = 0;
        data["hedef_yukseklik"] = 0;
        data["gps_saati"] = {
            {"saat", time.hour},
            {"dakika", time.minute},
            {"saniye", time.second},
            {"milisaniye", time.millisecond},
        };
    }



    void thread_func() {
        while (_is_running) {
            send_telemetry();
            std::this_thread::sleep_for(std::chrono::milliseconds(TELEM_INTERVAL));
        }
    }
};
}