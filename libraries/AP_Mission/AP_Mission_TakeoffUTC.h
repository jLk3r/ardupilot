#pragma once

#include <stdint.h>

// Local TAKEOFF_UTC wire contract: MAV_CMD_WAYPOINT_USER_1 (31000).
// p1 = UTC days since Unix epoch; p2 = integer UTC seconds of day;
// p3/p4/x/y = 0; z = climb above launch in metres; frame = GLOBAL_RELATIVE_ALT.
// A separate day avoids float32 Unix timestamp precision loss and next-day replay.
class AP_Mission_TakeoffUTC
{
public:
    enum class State : uint8_t { Idle, Waiting, Launch, Climbing, Airborne, Aborted };

    bool prepare(uint64_t now_utc_us, uint64_t now_us, uint64_t target_utc_us, uint16_t timeout_s)
    {
        _state = State::Aborted;
        if (now_utc_us == 0 || target_utc_us <= now_utc_us ||
            target_utc_us - now_utc_us > 86400000000ULL || timeout_s < 1 || timeout_s > 60) {
            return false;
        }
        _target_utc_us = target_utc_us;
        _target_us = now_us + target_utc_us - now_utc_us;
        _timeout_us = uint64_t(timeout_s) * 1000000;
        _state = State::Waiting;
        return true;
    }

    State update(uint64_t now_utc_us, uint64_t now_us, bool ready, bool armed, bool possibly_airborne)
    {
        if (_state == State::Waiting) {
            // Unexpected flight must never be treated as a fresh scheduled launch.
            const int64_t clock_error = int64_t(now_utc_us) - int64_t(_target_utc_us) -
                                        (int64_t(now_us) - int64_t(_target_us));
            if (now_utc_us == 0 || clock_error < -250000 || clock_error > 250000 || possibly_airborne) {
                return _state = State::Aborted;
            }
            if (now_utc_us < _target_utc_us) {
                return _state;
            }
            // Only normal scheduler latency is accepted, never a late retry.
            if (!ready || !armed || now_utc_us - _target_utc_us > 100000) {
                return _state = State::Aborted;
            }
            _target_us = now_us - (now_utc_us - _target_utc_us);
            return _state = State::Launch;
        }
        if (_state == State::Launch || _state == State::Climbing) {
            if (!armed) {
                return _state = State::Aborted;
            }
            // Conservative latch: a timeout must not disarm a possibly flying vehicle.
            if (possibly_airborne) {
                return _state = State::Airborne;
            }
            if (now_us - _target_us >= _timeout_us) {
                return _state = State::Aborted;
            }
            return _state = State::Climbing;
        }
        return _state;
    }

    State state() const
    {
        return _state;
    }
    void reset()
    {
        _state = State::Idle;
    }
    void abort()
    {
        _state = State::Aborted;
    }

private:
    State _state = State::Idle;
    uint64_t _target_utc_us = 0;
    uint64_t _target_us = 0;
    uint64_t _timeout_us = 0;
};
