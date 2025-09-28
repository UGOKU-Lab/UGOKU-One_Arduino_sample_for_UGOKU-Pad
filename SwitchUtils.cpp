#include "SwitchUtils.hpp"

struct SwState { const uint8_t* key; uint8_t prev; };
static SwState g_swStates[16];
static uint8_t g_swStateCount = 0;

void applySwitchPins(const uint8_t &swVar, std::initializer_list<int> pins) {
  uint8_t v = swVar;
  if (v == 0xFF) return; // 無効値は無視

  // 既存キー検索（SW変数のアドレス）
  uint8_t idx = 0xFF;
  for (uint8_t i = 0; i < g_swStateCount; ++i) {
    if (g_swStates[i].key == &swVar) { idx = i; break; }
  }
  // 未登録なら追加
  if (idx == 0xFF) {
    if (g_swStateCount < (uint8_t)(sizeof(g_swStates)/sizeof(g_swStates[0]))) {
      idx = g_swStateCount++;
      g_swStates[idx].key = &swVar;
      g_swStates[idx].prev = 0xFF;
    } else {
      // 追跡テーブルが満杯 → フォールバックで即時適用のみ
      const uint8_t lvl = (v == 1) ? LOW : HIGH;
      for (int p : pins) { digitalWrite(p, lvl); }
      return;
    }
  }

  if (v == g_swStates[idx].prev) return; // 変化なし
  g_swStates[idx].prev = v;

  const uint8_t lvl = (v == 1) ? LOW : HIGH;
  for (int p : pins) { digitalWrite(p, lvl); }
}
