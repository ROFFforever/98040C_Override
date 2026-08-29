#include "Telemetry/telemetry.h"
#include "pros/misc.hpp"
#include <cstdio>

Telemetry::~Telemetry() {
    if (file_) {
        std::fclose(file_);
    }
}

void Telemetry::setMode(Mode mode, const std::string& filename) {
    if (file_) {
        std::fclose(file_);
        file_ = nullptr;
    }

    mode_ = mode;

    if (mode_ == Mode::SDCard) {
        // PROS mounts the card's root as "/usd/" for normal file I/O (the
        // "/usd" path without the trailing folder is reserved for the
        // separate usd_list_files()-style API, not fopen()).
        if (!pros::usd::is_installed()) {
            mode_ = Mode::Wireless;
            return;
        }

        file_ = std::fopen(("/usd/" + filename).c_str(), "w");
        if (!file_) {
            mode_ = Mode::Wireless;
        }
    }
}

void Telemetry::send(const std::string& data) {
    if (mode_ == Mode::SDCard && file_) {
        std::fwrite(data.data(), 1, data.size(), file_);
        fflush(file_); // flush after every write so a match's worth of data
                        // survives even if the robot loses power before the
                        // file gets closed
        return;
    }

    printf("%s", data.c_str());
    fflush(stdout); // stdout isn't a real terminal on the V5, so it's fully
                     // buffered by default - without this, prints can just
                     // sit in the buffer instead of actually going out
}

void Telemetry::debug(const std::string& message) {
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

    send("{\"debug\": \"" + escaped + "\"}\n");
}
