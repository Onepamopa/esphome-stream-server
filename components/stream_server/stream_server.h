#pragma once

#include "esphome/core/component.h"
#include "esphome/components/socket/socket.h"
#include "esphome/components/uart/uart.h"

#ifdef USE_BINARY_SENSOR
#include "esphome/components/binary_sensor/binary_sensor.h"
#endif
#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif

#include <memory>
#include <string>
#include <vector>

#define PASSTHROUGH_IP150

class StreamServerComponent : public esphome::Component {
public:
    StreamServerComponent() = default;

    explicit StreamServerComponent(esphome::uart::UARTComponent *stream) : stream_{stream} {}

    void set_uart_parent(esphome::uart::UARTComponent *parent) { this->stream_ = parent; }
    void set_proxy_to(esphome::uart::UARTComponent *uart) { this->proxy_to_ = uart; }

    void set_buffer_size(size_t size) {
      this->primary_buf_size_ = size;
      this->secondary_buf_size_ = size;
    } // Primary & Secondary buf size
    //void secondary_set_buffer_size(size_t size) { this->secondary_buf_size_ = size; } // Secondary buf size

    #ifdef USE_BINARY_SENSOR
    void set_connected_sensor(esphome::binary_sensor::BinarySensor *connected) { this->connected_sensor_ = connected; }
    #endif
    #ifdef USE_SENSOR
    void set_connection_count_sensor(esphome::sensor::Sensor *connection_count) { this->connection_count_sensor_ = connection_count; }
    #endif

    void setup() override;
    void loop() override;
    void dump_config() override;
    void on_shutdown() override;

    float get_setup_priority() const override { return esphome::setup_priority::AFTER_WIFI; }

    void set_port(uint16_t port) { this->port_ = port; }

protected:
    void publish_sensor();

    void accept();
    void cleanup();
    void read();
    void flush();
    void write();

    struct Client {
        Client(std::unique_ptr<esphome::socket::Socket> socket, std::string identifier, size_t position);

        std::unique_ptr<esphome::socket::Socket> socket{nullptr};
        std::string identifier{};
        bool disconnected{false};
        size_t position{0};
    };

    esphome::uart::UARTComponent *stream_{nullptr}; // Primary UART
    esphome::uart::UARTComponent *proxy_to_{nullptr}; // Secondary UART

    size_t primary_buf_size_;
    std::unique_ptr<uint8_t[]> primary_buffer_{};
    size_t primary_buf_head_{0};
    size_t primary_buf_tail_{0};

    size_t buf_index(size_t pos) { return pos & (this->primary_buf_size_ - 1); }
    /// Return the number of consecutive elements that are ahead of @p pos in memory.
    size_t buf_ahead(size_t pos) { return (pos | (this->primary_buf_size_ - 1)) - pos + 1; }

    size_t secondary_buf_size_;
    std::unique_ptr<uint8_t[]> secondary_buffer_{};
    size_t secondary_buf_head_{0};
    size_t secondary_buf_tail_{0};

    size_t secondary_buf_index(size_t pos) { return pos & (this->secondary_buf_size_ - 1); }
    /// Return the number of consecutive elements that are ahead of @p pos in memory.
    size_t secondary_buf_ahead(size_t pos) { return (pos | (this->secondary_buf_size_ - 1)) - pos + 1; }

#ifdef USE_BINARY_SENSOR
    esphome::binary_sensor::BinarySensor *connected_sensor_;
#endif
#ifdef USE_SENSOR
    esphome::sensor::Sensor *connection_count_sensor_;
#endif

    uint16_t port_;
    std::unique_ptr<esphome::socket::Socket> socket_{};
    std::vector<Client> clients_{};
};
