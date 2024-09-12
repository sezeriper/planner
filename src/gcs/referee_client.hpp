#pragma once

#include "controller_mavlink.hpp"
#include "spdlog/spdlog.h"

#include <chrono>
#include <cpr/cpr.h>
#include <nlohmann/json.hpp>
#include <ctime>

using json = nlohmann::json;

namespace rota {
class referee_client {
    static constexpr int TELEM_INTERVAL = 900;
public:
    referee_client(const std::string& referee_server_ip, const std::string& team_name, const std::string& pswd, controller_mavlink& plane_controller) :
        _referee_server_ip(referee_server_ip), _team_name(team_name), _pswd(pswd), _mavlink{plane_controller}, _thread([this]() { thread_func(); }) {}

    ~referee_client() {
        _is_running = false;
        _thread.join();
    }

    void login() {
        json my_json = json::object({
            {"kadi", _team_name},
            {"sifre", _pswd},
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
        auto data = get_telem_data();
        data["takim_numarasi"] = _team_no;

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

        // auto now = std::chrono::system_clock::now();
        // std::uint64_t unix_epoch_time = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();

        std::time_t t = unix_epoch_time / 1000000ull;
        auto date = *std::gmtime(&t);

        spdlog::info("Unix epoch time: {}", unix_epoch_time);

        time_t time;
        time.hour = date.tm_hour;
        time.minute = date.tm_min;
        time.second = date.tm_sec;
        time.millisecond = (unix_epoch_time / 1000ull) % 1000ull;

        return time;
    };

    std::string _referee_server_ip;
    std::string _team_name;
    std::string _pswd;
    controller_mavlink& _mavlink;
    std::thread _thread;
    std::string _team_no;
    std::atomic_bool _is_running{true};

    real_t convert_yaw(real_t yaw) const {
        if (yaw < 0.0f) {
            return 360.0f + yaw;
        }
        else {
            return yaw;
        }
    }

    json get_telem_data() const {
        time_t time = get_time();

        json data;
        data["iha_enlem"] = _mavlink.get_coords().lat;
        data["iha_boylam"] = _mavlink.get_coords().lon;
        data["iha_irtifa"] = _mavlink.get_coords().alt;
        data["iha_dikilme"] = _mavlink.get_attitude().pitch;
        data["iha_yonelme"] = convert_yaw(_mavlink.get_attitude().yaw);
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
            {"milisaniye", time.millisecond}
        };

        return data;
    }

    void thread_func() {
        while (_is_running) {
            send_telemetry();
            std::this_thread::sleep_for(std::chrono::milliseconds(TELEM_INTERVAL));
        }
    }
};
}