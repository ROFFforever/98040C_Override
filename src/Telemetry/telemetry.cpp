#include "Telemetry/telemetry.h"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include <cstdio>

Telemetry::~Telemetry() {
    if (file_) {
        std::fclose(file_);
    }
}

bool Telemetry::setMode(Mode mode, const std::string& filename) {
    bool ok = true;

    fileMutex_.lock();
    if (file_) {
        std::fclose(file_);
        file_ = nullptr;
    }

    if (mode == Mode::SDCard) {
        // PROS mounts the card's root as "/usd/" for normal file I/O (the
        // "/usd" path without the trailing folder is reserved for the
        // separate usd_list_files()-style API, not fopen()).
        if (!pros::usd::is_installed()) {
            ok = false;
        } else {
            path_ = "/usd/" + filename;
            file_ = std::fopen(path_.c_str(), "w");
            if (!file_) {
                ok = false;
            }
            lastCommitMs_ = pros::millis();
            uncommitted_ = false;
        }
    }
    fileMutex_.unlock();

    Mode newMode = ok ? mode : Mode::Wireless;

    queueMutex_.lock();
    mode_ = newMode;
    pending_.clear();
    droppedLines_ = 0;
    queueMutex_.unlock();

    if (newMode == Mode::SDCard && writer_ == nullptr) {
        writer_ = new pros::Task([this] { writerLoop(); },
                                 TASK_PRIORITY_DEFAULT - 1, TASK_STACK_DEPTH_DEFAULT,
                                 "Telemetry SD writer");
    }
    return ok;
}

void Telemetry::setEnabled(Channel channel, bool enabled) {
    enabled_[(int)channel] = enabled;
}

bool Telemetry::isEnabled(Channel channel) {
    return enabled_[(int)channel];
}

void Telemetry::send(Channel channel, const std::string& data) {
    if (!isEnabled(channel)) {
        return;
    }
    write(data);
}

void Telemetry::write(const std::string& data) {
    queueMutex_.lock();
    bool toSD = (mode_ == Mode::SDCard);
    if (toSD) {
        if (pending_.size() + data.size() <= kMaxPendingBytes) {
            pending_ += data;
        } else {
            droppedLines_++;
        }
    }
    queueMutex_.unlock();

    if (toSD) {
        return;
    }

    printf("%s", data.c_str());
    fflush(stdout); // stdout isn't a real terminal on the V5, so it's fully
                     // buffered by default - without this, prints can just
                     // sit in the buffer instead of actually going out
}

void Telemetry::writerLoop() {
    std::string batch;

    while (true) {
        queueMutex_.lock();
        batch.swap(pending_);
        uint32_t dropped = droppedLines_;
        droppedLines_ = 0;
        queueMutex_.unlock();

        if (dropped > 0) {
            batch += "{\"debug\": \"telemetry dropped " + std::to_string(dropped) +
                     " lines - SD card fell behind\"}\n";
        }

        bool reopenFailed = false;

        fileMutex_.lock();
        if (file_ && !batch.empty()) {
            std::fwrite(batch.data(), 1, batch.size(), file_);
            fflush(file_); // flush after every write so a match's worth of data
                            // survives even if the robot loses power before the
                            // file gets closed
            uncommitted_ = true;
        }
        if (file_ && uncommitted_ && pros::millis() - lastCommitMs_ >= kCommitPeriodMs) {
            std::fclose(file_);
            file_ = std::fopen(path_.c_str(), "a");
            reopenFailed = (file_ == nullptr);
            uncommitted_ = false;
            lastCommitMs_ = pros::millis();
        }
        fileMutex_.unlock();

        if (reopenFailed) {
            queueMutex_.lock();
            mode_ = Mode::Wireless;
            pending_.clear();
            queueMutex_.unlock();
        }

        batch.clear();
        pros::delay(kWriterPeriodMs);
    }
}

namespace {
std::string escapeForJson(const std::string& message) {
    std::string escaped;
    escaped.reserve(message.size());

    for (char c : message) {
        switch (c) {
            case '"':  escaped += "\\\""; break;
            case '\\': escaped += "\\\\"; break;
            case '\n': escaped += "\\n";  break;
            case '\r': escaped += "\\r";  break;
            case '\t': escaped += "\\t";  break;
            default:   escaped += c;      break;
        }
    }

    return escaped;
}
}

void Telemetry::debug(const std::string& message) {
    send(Channel::Debug, "{\"debug\": \"" + escapeForJson(message) + "\"}\n");
}

void Telemetry::sendWireless(const std::string& data) {
    printf("%s", data.c_str());
    fflush(stdout);
}

void Telemetry::debugWireless(const std::string& message) {
    sendWireless("{\"debug\": \"" + escapeForJson(message) + "\"}\n");
}

bool Telemetry::appendToSD(const std::string& filename, const std::string& data) {
    if (!pros::usd::is_installed()) {
        return false;
    }

    fileMutex_.lock();
    std::FILE* f = std::fopen(("/usd/" + filename).c_str(), "a");
    if (f) {
        std::fwrite(data.data(), 1, data.size(), f);
        std::fclose(f);
    }
    fileMutex_.unlock();
    return f != nullptr;
}
