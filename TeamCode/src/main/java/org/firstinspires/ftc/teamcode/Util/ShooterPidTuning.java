package org.firstinspires.ftc.teamcode.Util;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import java.util.Map;
import java.util.WeakHashMap;

/**
 * Centralizes the shooter PIDF coefficients so they can be tweaked live from Panels.
 * Call {@link #applyTo(DcMotorEx)} anywhere the shooter is used to push the latest values.
 */
@Configurable(name = "Shooter PID")
public final class ShooterPidTuning {

    /** Default values close to the REV 2018-2019 PIDF set; adjust live from Panels. */
    public static double velocityKp = 1.17;
    public static double velocityKi = 0.117;
    public static double velocityKd = 0.0;
    public static double velocityKf = 11.7;

    private static final Map<DcMotorEx, PIDFCoefficients> appliedCoefficients = new WeakHashMap<>();
    private static final double EPSILON = 1e-6;

    private ShooterPidTuning() {
        // Utility class
    }

    /** Applies the currently configured PIDF values to the provided shooter motor. */
    public static void applyTo(DcMotorEx motor) {
        if (motor == null) {
            return;
        }

        PIDFCoefficients desired = new PIDFCoefficients(velocityKp, velocityKi, velocityKd, velocityKf);
        PIDFCoefficients current = appliedCoefficients.get(motor);
        if (!coefficientsEqual(current, desired)) {
            motor.setVelocityPIDFCoefficients(velocityKp, velocityKi, velocityKd, velocityKf);
            appliedCoefficients.put(motor, desired);
        }
    }

    private static boolean coefficientsEqual(PIDFCoefficients a, PIDFCoefficients b) {
        if (a == b) {
            return true;
        }
        if (a == null || b == null) {
            return false;
        }

        return Math.abs(a.p - b.p) < EPSILON
                && Math.abs(a.i - b.i) < EPSILON
                && Math.abs(a.d - b.d) < EPSILON
                && Math.abs(a.f - b.f) < EPSILON;
    }
}
