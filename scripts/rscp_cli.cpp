// rscp_cli.cpp — minimal CLI generator for RSCP frames
// Build with g++ -std=c++17 -O2 rscp_cli.cpp rscp.pb.c pb_common.c pb_encode.c -o rscp_cli


#include <iostream>
#include <vector>
#include <string>
#include <cstdint>
#include <cstdio>
#include "pb_encode.h"
#include "pb_common.h" 
#include "rscp.pb.h"

// === COBS encoder ===
size_t cobsEncode(const uint8_t* in, size_t len, uint8_t* out)
{
    const uint8_t* end = in + len;
    uint8_t* start = out;
    uint8_t* code_ptr = out++;
    uint8_t code = 1;

    while (in < end) {
        if (*in == 0) {
            *code_ptr = code;
            code_ptr = out++;
            code = 1;
            ++in;
        } else {
            *out++ = *in++;
            if (++code == 0xFF) {
                *code_ptr = code;
                code_ptr = out++;
                code = 1;
            }
        }
    }

    *code_ptr = code;
    *out++ = 0x00;
    return out - start;
}

int main(int argc, char* argv[])
{
    if (argc < 2) {
        std::cerr << "Usage: rscp_cli <command>\n";
        return 1;
    }

    std::string cmd = argv[1];
    rscp_RequestEnvelope req = rscp_RequestEnvelope_init_zero;

    if (cmd == "arm") {
        req.which_request = rscp_RequestEnvelope_arm_disarm_tag;
        req.request.arm_disarm.which_value_wrapper = rscp_ArmDisarm_value_tag;
        req.request.arm_disarm.value_wrapper.value = true;
        
    } else if (cmd == "disarm") {
        req.which_request = rscp_RequestEnvelope_arm_disarm_tag;
        req.request.arm_disarm.which_value_wrapper = rscp_ArmDisarm_value_tag;
        req.request.arm_disarm.value_wrapper.value = false;
        
    } else if (cmd == "setstage") {
        if (argc < 3) {
            std::cerr << "setstage needs a stage number (1-4)\n";
            return 1;
        }
        req.which_request = rscp_RequestEnvelope_set_stage_tag;
        req.request.set_stage.value = std::stoi(argv[2]);
        
    } else if (cmd == "goto" || cmd == "navigate") {
        if (argc < 4) {
            std::cerr << "Usage: rscp_cli goto <lat> <lon> [alt_m]\n";
            return 1;
        }

        double lat = std::stod(argv[2]);
        double lon = std::stod(argv[3]);
        float alt = (argc >= 5) ? std::stof(argv[4]) : 0.0f;

        req.which_request = rscp_RequestEnvelope_navigate_to_gps_tag;
        req.request.navigate_to_gps.has_coordinate = true;
        req.request.navigate_to_gps.coordinate.latitude = lat;
        req.request.navigate_to_gps.coordinate.longitude = lon;
        req.request.navigate_to_gps.coordinate.altitude = alt;

    } else if (cmd == "search") {
        if (argc < 5) {
            std::cerr << "Usage: rscp_cli search <lat> <lon> <radius_m> [alt_m]\n";
            return 1;
        }
        
        double lat = std::stod(argv[2]);
        double lon = std::stod(argv[3]);
        float radius = std::stof(argv[4]);
        float alt = (argc >= 6) ? std::stof(argv[5]) : 0.0f;
        
        req.which_request = rscp_RequestEnvelope_search_area_tag;
        req.request.search_area.has_center_coordinate = true;
        req.request.search_area.center_coordinate.latitude = lat;
        req.request.search_area.center_coordinate.longitude = lon;
        req.request.search_area.center_coordinate.altitude = alt;
        req.request.search_area.radius = radius;
        
    } else if (cmd == "explore" || cmd == "startexploration") {
        req.which_request = rscp_RequestEnvelope_start_exploration_tag;
        req.request.start_exploration.dummy_field = true;
    
    } else {
        std::cerr << "Unknown command\n";
        return 1;
    }

    // Encode and frame
    uint8_t pb_buf[128];
    pb_ostream_t os = pb_ostream_from_buffer(pb_buf, sizeof(pb_buf));
    if (!pb_encode(&os, rscp_RequestEnvelope_fields, &req)) {
        std::cerr << "pb_encode failed\n";
        return 1;
    }

    size_t pb_len = os.bytes_written;
    std::vector<uint8_t> frame(pb_len + pb_len / 254 + 2);
    size_t frame_len = cobsEncode(pb_buf, pb_len, frame.data());
    
    for (size_t i = 0; i < frame_len; ++i)
        printf("%02X", frame[i]);
    putchar('\n');
    return 0;
}
