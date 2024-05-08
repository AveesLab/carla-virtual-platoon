#include <map>
#include <array>

float IHP_loc[3][3] = {
    {-1139.0f,  4.5f, 1.0f},
    {-1169.0f,  4.5f, 1.0f},
    {-1199.0f,  4.5f, 1.0f}
};

float IHP_rot[3][3] = {
    {0.0f, 0.22f, 0.0f},
    {0.0f, 0.22f, 0.0f},
    {0.0f, 0.22f, 0.0f}
};


std::map<std::string, std::map<int, std::vector<float>>> truckLocations = {
    {"Town04", {
        {0, {-270.990f, 30.0f, 2.0f}},
        {1, {-300.990f, 30.0f, 2.0f}},
        {2, {-330.990f, 30.0f, 2.0f}}
    }},
    {"IHP", {
        {0, {-1139.0f, 4.5f, 1.0f}},
        {1, {-1169.0f, 4.5f, 1.0f}},
        {2, {-1199.0f, 4.5f, 1.0f}}
    }}
};

std::map<std::string, std::map<int, std::vector<float>>> truckRotations = {
    {"Town04", {
        {0, {0.0f, 0.22f, 0.0f}},
        {1, {0.0f, 0.22f, 0.0f}},
        {2, {0.0f, 0.22f, 0.0f}}
    }},
    {"IHP", {
        {0, {0.0f, 0.22f, 0.0f}},
        {1, {0.0f, 0.22f, 0.0f}},
        {2, {0.0f, 0.22f, 0.0f}}
    }}
};