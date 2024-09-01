#include <air_control.grpc.pb.h>
#include <grpcpp/grpcpp.h>

#include <spdlog/spdlog.h>

class air_control_client {
public:
    air_control_client(std::shared_ptr<grpc::Channel> channel) :
        _stub(air_control::NewStub(channel)) {}

    void record_trajectory(const std::vector<configuration_t>& trajectory)
    {
        class recorder : public grpc::ClientWriteReactor<configuration_t> {
        public:
            recorder(air_control::Stub* stub, const std::vector<configuration_t>* trajectory) :
                _trajectory(trajectory), _index{0}, _done{false}
            {
                stub->async()->record_trajectory(&_context, &_empty, this);

                AddHold();
                StartWrite(&_trajectory->operator[](_index));
                StartCall();
            }

            void OnWriteDone(bool) override {
                if (_index == _trajectory->size()) {
                    StartWritesDone();
                    RemoveHold();
                    return;
                }

                StartWrite(&_trajectory->operator[](_index));
                ++_index;
            }

            void OnDone(const grpc::Status& s) override {
                std::unique_lock<std::mutex> lock(_mutex);
                _done = true;
                _cv.notify_one();
                spdlog::info("RPC completed");
            }

            void Await() {
                std::unique_lock<std::mutex> lock(_mutex);
                _cv.wait(lock, [this] { return _done; });
                return;
            }

        private:
            const std::vector<configuration_t>* _trajectory;
            std::size_t _index;
            bool _done;

            empty_t _empty;
            grpc::ClientContext _context;
            std::mutex _mutex;
            std::condition_variable _cv;
        };
        recorder recorder(_stub.get(), &trajectory);
        recorder.Await();
    }

private:
    std::unique_ptr<air_control::Stub> _stub;
};


int main() {
    air_control_client ac(
        grpc::CreateChannel("localhost:1923", grpc::InsecureChannelCredentials()));

    point_t* point = new point_t;
    point->set_x(0.0f);
    point->set_y(0.0f);

    configuration_t conf;
    conf.set_allocated_position(point);
    conf.set_yaw(90.0f);

    std::vector<configuration_t> trajectory{conf};

    ac.record_trajectory(trajectory);

    return 0;
}