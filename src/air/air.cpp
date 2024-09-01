#include <air_control.grpc.pb.h>
#include <grpcpp/grpcpp.h>

#include <spdlog/spdlog.h>

using namespace grpc;


class air_control_impl final : public air_control::CallbackService {
    ServerReadReactor<configuration_t>* record_trajectory(
        CallbackServerContext* context, empty_t* empty) override {

            class recorder : public ServerReadReactor<configuration_t> {
            public:
                recorder(empty_t* empty, std::vector<configuration_t>* trajectory) :
                    _trajectory(trajectory)
                    { StartRead(&_configuration); }

                    void OnReadDone(bool ok) override {
                        if (ok) {
                            _trajectory->push_back(_configuration);
                            StartRead(&_configuration);
                            return;
                        }
                        Finish(Status::OK);
                    }

                    void OnDone() override {
                        spdlog::info("RPC completed.");

                        for (const auto& config : *_trajectory) {
                            spdlog::info("Received configuration: (({}, {}), {})", config.position().x(), config.position().y(), config.yaw());
                        }
                        delete this;
                    }

                    void OnCancel() override {
                        spdlog::error("RPC cancelled.");
                    }
            private:
                std::vector<configuration_t>* _trajectory;
                configuration_t _configuration;
            };
            return new recorder(empty, &_trajectory);
        }
    
private:
    std::vector<configuration_t> _trajectory;
};

void run_server() {
    std::string server_address{"0.0.0.0:1923"};
    air_control_impl service;
    ServerBuilder builder;

    builder.AddListeningPort(server_address, InsecureServerCredentials());
    builder.RegisterService(&service);
    std::unique_ptr<Server> server(builder.BuildAndStart());
    spdlog::info("Server listening on {}", server_address);
    server->Wait();
    spdlog::info("Server shutting down.");
}

int main() {
    run_server();
    return 0;
}