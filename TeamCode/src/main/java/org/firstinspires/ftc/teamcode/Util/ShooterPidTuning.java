package org.firstinspires.ftc.teamcode.Util;

import static com.qualcomm.robotcore.hardware.MotorControlAlgorithm.PIDF;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * Centralizes the shooter PIDF coefficients so they can be tweaked live from Panels.
 * Call {@link #applyTo(DcMotorEx)} anywhere the shooter is used to push the latest values.
 */
@Configurable
public final class ShooterPidTuning {

    /** Default values aligned with existing shooter PID settings; adjust live from Panels. */
    public static double velocityKp = 20.0; // REV 2018-2019 PIDF Kp = 1.17
    public static double velocityKi = 0.5;  // REV 2018-2019 PIDF Ki = 0.117
    public static double velocityKd = 0.0; // REV 2018-2019 PIDF Kd = 0.0
    public static double velocityKf = 16.0;  // REV 2018-2019 PIDF Kf = 11.7

    private static final double EPSILON = 1e-6;

    private ShooterPidTuning() {
        // Utility class
    }

    /** Applies the currently configured PIDF values to the provided shooter motor. */
    public static void applyTo(DcMotorEx motor) {
        if (motor == null) {
            return;
        }

        PIDFCoefficients desired = new PIDFCoefficients(velocityKp, velocityKi, velocityKd, velocityKf, PIDF);
        // Pull the live coefficients from the hub so we can recover if the hub resets mid‑match.
        PIDFCoefficients current = motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        if (!coefficientsEqual(current, desired)) {
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, desired);
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
                && Math.abs(a.f - b.f) < EPSILON
                && a.algorithm == b.algorithm;
    }
}
