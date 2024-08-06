#include <map>
#include <array>

std::map<std::string, std::map<int, std::vector<float>>> truckLocations = {
    {"Town04_Opt", {
        {0, {-270.990f, 27.0f, 2.0f}},
        {1, {-300.990f, 27.0f, 2.0f}},
        {2, {-330.990f, 27.0f, 2.0f}}
    }},
    {"IHP", {
        {0, {-1139.0f, 4.5f, 1.0f}},
        {1, {-1169.0f, 4.5f, 1.0f}},
        {2, {-1199.0f, 4.5f, 1.0f}}
    }},
    {"k-track", {
        {0, {-75.6f,  50.75f,  2.0f}},
        {1, {-100.6f, 50.75f, 2.0f}},
        {2, {-133.6f, 50.75f, 2.0f}}
    }

    }

};

std::map<std::string, std::map<int, std::vector<float>>> truckRotations = {
    {"Town04_Opt", {
        {0, {0.0f, 0.0f, 0.0f}},
        {1, {0.0f, 0.22f, 0.0f}},
        {2, {0.0f, 0.22f, 0.0f}}
    }},
    {"IHP", {
        {0, {0.0f, 0.0f, 0.0f}},
        {1, {0.0f, 0.0f, 0.0f}},
        {2, {0.0f, 0.0f, 0.0f}}
    }},
    {"k-track", {
        {0, {0.0f, 0.0f, 0.0f}},
        {1, {0.0f, 0.0f, 0.0f}},
        {2, {0.0f, 0.0f, 0.0f}}
    }}
};