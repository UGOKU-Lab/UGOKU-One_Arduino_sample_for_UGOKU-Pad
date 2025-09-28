#pragma once

#include <stdint.h>
#include <initializer_list>
#include <Arduino.h>

// 任意のSW変数(SW1, SW6 など)を指定し、ピン群を 1->LOW, それ以外->HIGH に一括適用
// 変化時のみ出力するため、SW変数のアドレスごとに前回値を内部で保存します。
void applySwitchPins(const uint8_t &swVar, std::initializer_list<int> pins);

// 使いやすい可変長版: applySwitchPins(SW1, PIN1, PIN2, ...)
template <typename... Pins>
static inline void applySwitchPins(const uint8_t &swVar, Pins... pins) {
  applySwitchPins(swVar, std::initializer_list<int>{ static_cast<int>(pins)... });
}
