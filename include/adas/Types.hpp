#pragma once

#include <opencv2/core.hpp>

#include <cstdint>
#include <string_view>

namespace adas {

enum class ObjectClass : std::uint8_t {
    Person,
    Bicycle,
    Car,
    Motorcycle,
    Bus,
    Truck
};


struct Detection {
    cv::Rect box;                        // x, y, širina, visina u pikselima ORIGINALNOG fram
    float confidence{0.0F};              
    ObjectClass cls{ObjectClass::Car};   
};

// Razina opasnosti koju računa AdasLogic
enum class RiskLevel : std::uint8_t {
    None,     
    Caution,  
    Warning   
};


[[nodiscard]] constexpr std::string_view toString(ObjectClass c) noexcept {
    switch (c) {
        case ObjectClass::Person:     return "person";
        case ObjectClass::Bicycle:    return "bicycle";
        case ObjectClass::Car:        return "car";
        case ObjectClass::Motorcycle: return "motorcycle";
        case ObjectClass::Bus:        return "bus";
        case ObjectClass::Truck:      return "truck";
    }
    return "unknown";
}


[[nodiscard]] constexpr bool isVulnerable(ObjectClass c) noexcept {
    return c == ObjectClass::Person ||
           c == ObjectClass::Bicycle ||
           c == ObjectClass::Motorcycle;
}

}  