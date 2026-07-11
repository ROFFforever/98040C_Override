#include "Telemetry/telemetry.h"
#include <cstdio>

void Telemetry::send(const std::string& data) {
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
