"""
Deterministic runway-conflict scenario to demonstrate arrival priority.

Flow:
- One arrival is 30s from the threshold and will be cleared to land.
- One departure is waiting at the hold-short line.
- Departure requests takeoff every tick but will only be cleared if the
  runway can be vacated before the arrival reaches the threshold.

Run: python runway_conflict_scenario.py
"""

import math
from typing import Optional

from aircraft import EmbraerE170, AirbusA220
from ground_operations import GroundOperations, RUNWAY_LENGTH


def log(t: float, msg: str):
    print(f"[t={t:5.1f}s] {msg}")


def simulate_conflict():
    ops = GroundOperations(runway_length=RUNWAY_LENGTH, num_gates=19)

    arrival = EmbraerE170("ARR-101")
    departure = AirbusA220("DEP-201")

    # Arrival ETA to threshold (seconds from now)
    arrival_eta = 30.0
    current_time = 0.0
    dt = 1.0

    # Track if events have been triggered
    departure_started = False
    arrival_announced = False

    log(current_time, f"Arrival {arrival.id} inbound, ETA {arrival_eta:.1f}s")
    log(current_time, f"Departure {departure.id} holding short, requesting takeoff when safe")

    # Seed inbound ETA tracking so departures can see the conflict window
    ops.update_arrival_eta(arrival, arrival_eta, current_time)

    # Keep the sim short and deterministic
    end_time = 120.0

    while current_time <= end_time:
        # Update inbound ETA countdown
        if arrival in ops.inbound_arrivals:
            remaining = max(0.0, ops.inbound_arrivals[arrival]["eta"] - current_time)
        else:
            remaining = None

        # Trigger landing clearance when "on final" (15s before ETA here)
        if remaining is not None and remaining <= 15.0 and arrival not in ops.landing_clearances:
            cleared = ops.request_landing_clearance(arrival, current_time, on_final_approach=True)
            if cleared:
                log(current_time, f"ATC: {arrival.id} CLEARED TO LAND (runway blocked until t={ops.runway_free_time:.1f})")

        # Check if the runway can be used for takeoff
        if not departure_started:
            takeoff_time = ops.estimate_takeoff_runway_time(departure)
            if not ops.is_runway_available(current_time):
                pass  # busy; nothing to log every tick
            elif ops.has_arrival_conflict(current_time, takeoff_time):
                # Arrival is close enough that a takeoff would conflict
                if math.isclose(current_time % 5.0, 0.0, abs_tol=1e-6):
                    eta = ops.get_next_arrival_eta(current_time)
                    wait = 0.0 if eta is None else max(0.0, eta - current_time)
                    log(current_time, f"Takeoff HOLD for arrival (ETA {wait:.1f}s), need ~{takeoff_time:.1f}s to clear")
            else:
                ops.mark_runway_busy(departure, takeoff_time, current_time)
                departure_started = True
                log(current_time, f"ATC: {departure.id} CLEARED FOR TAKEOFF (blocking {takeoff_time:.1f}s)")

        # Release runway automatically when busy timer expires
        if ops.runway_busy and current_time >= ops.runway_free_time:
            log(current_time, f"Runway now FREE (previous user {ops.current_runway_plane.id if ops.current_runway_plane else 'N/A'})")
            ops.release_runway()
            # When the arrival has landed, drop its ETA tracking
            if arrival in ops.inbound_arrivals:
                ops.clear_arrival_eta(arrival)

        # Arrival reaches the threshold at its ETA if still tracked
        if remaining is not None and remaining <= 0.0 and arrival in ops.inbound_arrivals and not arrival_announced:
            log(current_time, f"{arrival.id} has reached the threshold")
            arrival_announced = True
            # If not already marked busy (should be), mark a short rollout window
            if not ops.runway_busy:
                ops.mark_runway_busy(arrival, 40.0, current_time)
                log(current_time, f"Runway was free unexpectedly; blocking for landing until t={ops.runway_free_time:.1f}")

        current_time += dt


if __name__ == "__main__":
    simulate_conflict()
