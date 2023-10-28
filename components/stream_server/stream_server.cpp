#include "stream_server.h"

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include "esphome/core/util.h"
#include "esphome/core/version.h"

#include "esphome/components/network/util.h"
#include "esphome/components/socket/socket.h"

static const char *TAG = "stream_server";

using namespace esphome;

void StreamServerComponent::setup() {
    ESP_LOGCONFIG(TAG, "Setting up stream server...");

    // The make_unique() wrapper doesn't like arrays, so initialize the unique_ptr directly.
    this->primary_buffer_ = std::unique_ptr<uint8_t[]>{new uint8_t[this->primary_buf_size_]};
    this->secondary_buffer_ = std::unique_ptr<uint8_t[]>{new uint8_t[this->secondary_buf_size_]};

    struct sockaddr_storage bind_addr;
    #if ESPHOME_VERSION_CODE >= VERSION_CODE(2023, 4, 0)
    socklen_t bind_addrlen = socket::set_sockaddr_any(reinterpret_cast<struct sockaddr *>(&bind_addr), sizeof(bind_addr), this->port_);
    #else
    socklen_t bind_addrlen = socket::set_sockaddr_any(reinterpret_cast<struct sockaddr *>(&bind_addr), sizeof(bind_addr), htons(this->port_));
    #endif

    this->socket_ = socket::socket_ip(SOCK_STREAM, PF_INET);
    this->socket_->setblocking(false);
    this->socket_->bind(reinterpret_cast<struct sockaddr *>(&bind_addr), bind_addrlen);
    this->socket_->listen(8);

    this->publish_sensor();
}

void StreamServerComponent::loop() {
    //this->accept();
    this->read();
    //this->flush();
    //this->write();
    //this->cleanup();
}

void StreamServerComponent::dump_config() {
    ESP_LOGCONFIG(TAG, "Stream Server:");
    ESP_LOGCONFIG(TAG, "  Address: %s:%u", esphome::network::get_use_address().c_str(), this->port_);
#ifdef USE_BINARY_SENSOR
    LOG_BINARY_SENSOR("  ", "Connected:", this->connected_sensor_);
#endif
#ifdef USE_SENSOR
    LOG_SENSOR("  ", "Connection count:", this->connection_count_sensor_);
#endif
}

void StreamServerComponent::on_shutdown() {
    for (const Client &client : this->clients_)
        client.socket->shutdown(SHUT_RDWR);
}

void StreamServerComponent::publish_sensor() {
#ifdef USE_BINARY_SENSOR
    if (this->connected_sensor_)
        this->connected_sensor_->publish_state(this->clients_.size() > 0);
#endif
#ifdef USE_SENSOR
    if (this->connection_count_sensor_)
        this->connection_count_sensor_->publish_state(this->clients_.size());
#endif
}

/**
 * Accept new TCP client
 */
void StreamServerComponent::accept() {
    struct sockaddr_storage client_addr;
    socklen_t client_addrlen = sizeof(client_addr);
    std::unique_ptr<socket::Socket> socket = this->socket_->accept(reinterpret_cast<struct sockaddr *>(&client_addr), &client_addrlen);
    if (!socket)
        return;

    socket->setblocking(false);
    std::string identifier = socket->getpeername();
    this->clients_.emplace_back(std::move(socket), identifier, this->primary_buf_head_);
    ESP_LOGD(TAG, "New client connected from %s", identifier.c_str());
    this->publish_sensor();
}

#define MOD_RX_TX

/**
 * Read from the Primary UART, write to Secondary
 * Read from Secondary UART, write to Primary
 */
void StreamServerComponent::read() {
    size_t primary_len = 0;
    int available;

    // ------------------------------------ Primary UART ------------------------------------
    // Read, write to secondary
    while ((available = this->stream_->available()) > 0) {
        size_t free = this->primary_buf_size_ - (this->primary_buf_head_ - this->primary_buf_tail_);
        
        if (free == 0) {
            // Only overwrite if nothing has been added yet, otherwise give flush() a chance to empty the buffer first.
            if (primary_len > 0)
                return;

            ESP_LOGE(TAG, "Incoming bytes available, but outgoing buffer is full: stream will be corrupted!");
            free = std::min<size_t>(available, this->primary_buf_size_);
            this->primary_buf_tail_ += free;
            for (Client &client : this->clients_) {
                if (client.position < this->primary_buf_tail_) {
                    ESP_LOGW(TAG, "Dropped %u pending bytes for client %s", this->primary_buf_tail_ - client.position, client.identifier.c_str());
                    client.position = this->primary_buf_tail_;
                }
            }
        }

        // Fill all available contiguous space in the ring buffer.
        primary_len = std::min<size_t>(available, std::min<size_t>(this->buf_ahead(this->primary_buf_head_), free));
        this->stream_->read_array(&this->primary_buffer_[this->buf_index(this->primary_buf_head_)], primary_len);

        this->primary_buf_head_ += primary_len;

        #ifndef MOD_RX_TX
        // Write to the secondary uart
        this->proxy_to_->write_array(&this->primary_buffer_[this->buf_index(this->primary_buf_head_)], primary_len);
        #endif // MOD_RX_TX
    }

    #ifdef MOD_RX_TX
    if (available) {
        // Write to the secondary uart
        this->proxy_to_->write_array(&this->primary_buffer_[this->buf_index(this->primary_buf_head_)], primary_len);
    }
    #endif // MOD_RX_TX

    // ------------------------------------ Secondary UART ------------------------------------
    // Read, write to primary
    size_t secondary_len = 0;
    int secondary_available;

    while ((secondary_available = this->proxy_to_->available()) > 0) {
      size_t free = this->secondary_buf_size_ - (this->secondary_buf_head_ - this->secondary_buf_tail_);

        // Fill all available contiguous space in the ring buffer.
        secondary_len = std::min<size_t>(secondary_available, std::min<size_t>(this->secondary_buf_ahead(this->secondary_buf_head_), free));
        this->proxy_to_->read_array(&this->secondary_buffer_[this->secondary_buf_index(this->secondary_buf_head_)], secondary_len);

        this->secondary_buf_head_ += secondary_len;

        #ifndef MOD_RX_TX
        // Write to the primary uart
        this->stream_->write_array(&this->secondary_buffer_[this->secondary_buf_index(this->secondary_buf_head_)], secondary_len);
        #endif // MOD_RX_TX
    }

    if (secondary_available) {
        // Write to the primary uart
        this->stream_->write_array(&this->secondary_buffer_[this->secondary_buf_index(this->secondary_buf_head_)], secondary_len);
    }
}

/**
 * Write to TCP client
 */
void StreamServerComponent::flush() {
    ssize_t written;
    
    this->primary_buf_tail_ = this->primary_buf_head_;
    this->secondary_buf_tail_ = this->secondary_buf_head_;

    /*
    for (Client &client : this->clients_) {
        if (client.disconnected || client.position == this->primary_buf_head_) {
            // If the client is disconnected, or has already received all data, pass
            continue;
        }

        // Split the write into two parts: from the current position to the end of the ring buffer, and from the start
        // of the ring buffer until the head. The second part might be zero if no wraparound is necessary.
        struct iovec iov[2];
        iov[0].iov_base = &this->primary_buffer_[this->buf_index(client.position)];
        iov[0].iov_len = std::min(this->primary_buf_head_ - client.position, this->buf_ahead(client.position));
        iov[1].iov_base = &this->primary_buffer_[0];
        iov[1].iov_len = this->primary_buf_head_ - (client.position + iov[0].iov_len);

        if ((written = client.socket->writev(iov, 2)) > 0) {
            client.position += written;
        } else if (written == 0 || errno == ECONNRESET) {
            ESP_LOGD(TAG, "Client %s disconnected", client.identifier.c_str());
            client.disconnected = true;
            continue;  // don't consider this client when calculating the tail position
        } else if (errno == EWOULDBLOCK || errno == EAGAIN) {
            // Expected if the (TCP) transmit buffer is full, nothing to do.
        } else {
            ESP_LOGE(TAG, "Failed to write to client %s with error %d!", client.identifier.c_str(), errno);
        }

        this->primary_buf_tail_ = std::min(this->primary_buf_tail_, client.position);
    }
    */
}

/**
 * Read from TCP clients, write to Primary UART
 */
void StreamServerComponent::write() {
    uint8_t buf[this->primary_buf_size_]; // 128 -> primary_buf_size_ to match what's been set
    
    ssize_t read;
    for (Client &client : this->clients_) {
        if (client.disconnected)
            continue;

        while ((read = client.socket->read(&buf, sizeof(buf))) > 0)
            this->stream_->write_array(buf, read);

        if (read == 0 || errno == ECONNRESET) {
            ESP_LOGD(TAG, "Client %s disconnected", client.identifier.c_str());
            client.disconnected = true;
        } else if (errno == EWOULDBLOCK || errno == EAGAIN) {
            // Expected if the (TCP) receive buffer is empty, nothing to do.
        } else {
            ESP_LOGW(TAG, "Failed to read from client %s with error %d!", client.identifier.c_str(), errno);
        }
    }
}

/**
 * Client cleanup
 */
void StreamServerComponent::cleanup() {
    auto discriminator = [](const Client &client) { return !client.disconnected; };
    auto last_client = std::partition(this->clients_.begin(), this->clients_.end(), discriminator);
    if (last_client != this->clients_.end()) {
        this->clients_.erase(last_client, this->clients_.end());
        this->publish_sensor();
    }
}

StreamServerComponent::Client::Client(std::unique_ptr<esphome::socket::Socket> socket, std::string identifier, size_t position)
    : socket(std::move(socket)), identifier{identifier}, position{position} {}
