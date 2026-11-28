package org.firstinspires.ftc.teamcode;

/**
 * Timing for the autos: start lowering the FlowerRamp during the drive to the flower so it reaches the floor just
 * before the intake starts. FlowerRamp.java itself is unchanged; this only decides WHEN to call ramp.lower().
 *
 * Each loop while driving to the flower an auto asks shouldLower(remainingDistance, speed). Once the time left
 * equals the ramp's travel time plus MARGIN_S, it returns true. On arrival the auto waits for ready(ramp)
 * before turning the intake on.
 */
public class FlowerRampTiming {
    static final double MARGIN_S = 0.1;    // finish this long before the robot arrives
    static final double MIN_LEAD_IN = 12;  // always lower within this many inches of the flower, even if the speed reads low near the end

    /** Seconds the ramp needs to go from up to the floor and back off to the hover spot (FlowerRamp moves SPEED units per second). */
    static double lowerSeconds() {
        double travel = Math.abs(FlowerRamp.DOWN_POS - FlowerRamp.UP_POS);
        return (travel + Math.min(FlowerRamp.HOVER_POS, travel)) / FlowerRamp.SPEED;
    }

    /** True once the robot is close enough to the flower that the ramp must start down now. */
    static boolean shouldLower(double remainingIn, double speedInPerSec) {
        double leadIn = Math.abs(speedInPerSec) * (lowerSeconds() + MARGIN_S);
        return remainingIn <= Math.max(leadIn, MIN_LEAD_IN);
    }

    /** True when the ramp has touched the floor and settled at the hover spot, or the positions are not set up (nothing to wait for). */
    static boolean ready(FlowerRamp ramp) {
        return FlowerRamp.UP_POS == FlowerRamp.DOWN_POS || ramp.isDown();
    }
}
