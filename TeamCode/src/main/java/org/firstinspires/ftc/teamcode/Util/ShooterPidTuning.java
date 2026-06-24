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
    //public static double velocityKp = 20.0; // REV 2018-2019 PIDF Kp = 1.17
    public static double velocityKp = 45; // REV 2018-2019 PIDF Kp = 1.17
    //public static double velocityKi = 0.0;  // REV 2018-2019 PIDF Ki = 0.117
    public static double velocityKi = 0;  // REV 2018-2019 PIDF Ki = 0.117
    //public static double velocityKd = 0.0; // REV 2018-2019 PIDF Kd = 0.0
    public static double velocityKd = 10; // REV 2018-2019 PIDF Kd = 0.0
    //public static double velocityKf = 18.0;  // REV 2018-2019 PIDF Kf = 11.7
    public static double velocityKf = 17.8;  // REV 2018-2019 PIDF Kf = 11.7

    public static double FrVelocityKp = 60;
    public static double FrVelocityKi = 0;
    public static double FrVelocityKd = 0;
    public static double FrVelocityKf = 20;

    public static double BVelocityKp = 60;
    public static double BVelocityKi = 0;
    public static double BVelocityKd = 5;
    public static double BVelocityKf = 15.2;

    private static final double EPSILON = 1e-6;

    private ShooterPidTuning() {
        // Utility class
    }

    /** Applies the currently configured PIDF values to the provided shooter motor. */
    public static boolean applyTo(DcMotorEx motor) {
        if (motor == null) {
            return false;
        }

        PIDFCoefficients desired = new PIDFCoefficients(velocityKp, velocityKi, velocityKd, velocityKf, PIDF);
        // Pull the live coefficients from the hub so we can recover if the hub resets mid‑match.
        PIDFCoefficients current = motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        if (!coefficientsEqual(current, desired)) {
            // Use both velocity-specific and general setters to catch SDK/hub quirks.
            motor.setVelocityPIDFCoefficients(velocityKp, velocityKi, velocityKd, velocityKf);
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, desired);
        }
        current = motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        return coefficientsEqual(current,desired);
    }

    /**
     * applyTo(DcMotorEx motor, double set) applies the PIDF values to the given motor
     * @param motor - The motor the values are applied to
     * @param set - Chooses the set of values to apply
     *            - 1; General Values
     *            - 2; ShooterFront Specific values
     *            - 3; ShooterBack Specific values
    **/
    public static boolean applyTo(DcMotorEx motor, double set) {
        if (motor == null) {
            return false;
        }
        boolean result = false;
        if (set == 1) {
            PIDFCoefficients desired = new PIDFCoefficients(velocityKp, velocityKi, velocityKd, velocityKf, PIDF);
            // Pull the live coefficients from the hub so we can recover if the hub resets mid‑match.
            PIDFCoefficients current = motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            if (!coefficientsEqual(current, desired)) {
                // Use both velocity-specific and general setters to catch SDK/hub quirks.
                motor.setVelocityPIDFCoefficients(velocityKp, velocityKi, velocityKd, velocityKf);
                motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, desired);
            }
            current = motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            result = coefficientsEqual(current,desired);
        } else if (set == 2) {
            PIDFCoefficients desired = new PIDFCoefficients(FrVelocityKp, FrVelocityKi, FrVelocityKd, FrVelocityKf, PIDF);
            // Pull the live coefficients from the hub so we can recover if the hub resets mid‑match.
            PIDFCoefficients current = motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            if (!coefficientsEqual(current, desired)) {
                // Use both velocity-specific and general setters to catch SDK/hub quirks.
                motor.setVelocityPIDFCoefficients(FrVelocityKp, FrVelocityKi, FrVelocityKd, FrVelocityKf);
                motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, desired);
            }
            current = motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            result = coefficientsEqual(current,desired);
        } else if (set == 3)
        {
            PIDFCoefficients desired = new PIDFCoefficients(BVelocityKp, BVelocityKi, BVelocityKd, BVelocityKf, PIDF);
            // Pull the live coefficients from the hub so we can recover if the hub resets mid‑match.
            PIDFCoefficients current = motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            if (!coefficientsEqual(current, desired)) {
                // Use both velocity-specific and general setters to catch SDK/hub quirks.
                motor.setVelocityPIDFCoefficients(BVelocityKp, BVelocityKi, BVelocityKd, BVelocityKf);
                motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, desired);
            }
            current = motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            result = coefficientsEqual(current,desired);
        }
        return result;
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
