#pragma once
#include <Arduino.h>

/**
 * TimeBudgetManager
 *
 * Manages a time budget within a repeating slot window.
 * Example: "Allow at most 1 minute ON per 10-minute slot".
 * 
 * Call tick() every loop to keep timers updated.
 * Use isAllowed() to check if the budget allows turning on.
 */

class TimeBudgetManager {
public:
  /**
   * Constructor.
   * @param slotDurationMs Length of each slot window in milliseconds.
   * @param budgetDurationMs Maximum ON time allowed per slot in milliseconds.
   */
  TimeBudgetManager(unsigned long slotDurationMs, unsigned long budgetDurationMs)
    : SLOT_DURATION(slotDurationMs),
      BUDGET_DURATION(budgetDurationMs),
      m_slotTimer(slotDurationMs),
      m_budgetTimer(budgetDurationMs)
  {
    m_lastTick = millis();
  }

  /**
   * Call every loop to update timers.
   * @param isOn Pass true if the device is currently ON.
   */
  void tick(bool isOn) {
    unsigned long now = millis();
    unsigned long delta = now - m_lastTick;
    m_lastTick = now;

    // Countdown the slot timer
    if (m_slotTimer > 0) {
      m_slotTimer -= delta;
      if (m_slotTimer < 0) m_slotTimer = 0;
    }

    // If ON, countdown budget and accumulate daily usage
    if (isOn) {
      if (m_budgetTimer > 0) {
        m_budgetTimer -= delta;
        if (m_budgetTimer < 0) m_budgetTimer = 0;
      }
      m_dailyUsed += delta;
    }

    // If slot has expired, reset slot and budget for new cycle
    if (m_slotTimer == 0) {
      m_slotTimer = SLOT_DURATION;
      m_budgetTimer = BUDGET_DURATION;
    }
  }

  /**
   * Check if you are allowed to turn ON now.
   * @return True if enough budget remains in the current slot.
   */
  bool isAllowed() const {
    return m_budgetTimer > 0;
  }

  /**
   * Get total ON time used since last daily reset.
   * Units: milliseconds.
   */
  unsigned long getDailyUsed() const {
    return m_dailyUsed;
  }

  /**
   * Get remaining time in the current slot.
   * Units: milliseconds.
   */
  unsigned long getRemainingSlotMs() const {
    return m_slotTimer;
  }

  /**
   * Get remaining ON time available in the current slot.
   * Units: milliseconds.
   */
  unsigned long getRemainingBudgetMs() const {
    return m_budgetTimer;
  }

  /**
   * Optional: call this at midnight to reset daily usage.
   */
  void resetDailyUsed() {
    m_dailyUsed = 0;
  }

  void printStatus(const String& label = "") const {
    Serial.printf("[%sBudget] Slot remaining: %lu ms | Budget remaining: %lu ms | Daily used: %lu min\n",
        label.c_str(),
        m_slotTimer,
        m_budgetTimer,
        m_dailyUsed / 60000UL  // ms → min
    );
  }

private:
  // Configured slot and budget durations
  const unsigned long SLOT_DURATION;
  const unsigned long BUDGET_DURATION;

  // Remaining time in current slot and ON-time budget
  long m_slotTimer;
  long m_budgetTimer;

  // For delta time calculations
  unsigned long m_lastTick;

  // Total ON time used in the current day
  unsigned long m_dailyUsed = 0;
};
