package org.firstinspires.ftc.teamcode.CleanUp;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Util.Constants;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import static com.qualcomm.robotcore.util.TypeConversion.byteArrayToInt;
public class Shooter {
    public static double shooterTimer;
    public static int shooterState;
    public static int shotCount;
    private static int prevShotCount;

    public static double shooterFTarget;
    public static double shooterBTarget;

    public static boolean isAuto;

    /**
     * Resets the shot count tracking
     */
    public static void resetShotCount(){
        shotCount = 0;
    }


    /**
     * Shooter process (for all uses of shooter)
     * Set motor speeds to interpolated target speed
     * if inside range, run passthrough and intake
     * @param shooterFMotor Front Motor
     * @param shooterBMotor Back Motor
     * @param intake Intake Motor
     * @param shooterTarget Target Range
     * @param runtime Current Runtime
     * @param Passthrough Passthrough Motor
     * @param shootRequest Try to Shoot?
     * @param revRequest Try to Rev?
     */
    public static void runShooter(DcMotorEx shooterFMotor,
                                  DcMotorEx shooterBMotor,
                                  DcMotorEx intake,
                                  double shooterTarget,
                                  double runtime,
                                  CRServo Passthrough,
                                  boolean shootRequest,
                                  boolean revRequest
    )
    {
        //Read current shooter speeds
        double ShooterFspeed = shooterFMotor.getVelocity();
        double ShooterBspeed = shooterBMotor.getVelocity();

        if (shooterTarget < 1.06){
            shooterFTarget = 0;
            shooterBTarget = 0;
            shooterState = 0;
        } else if (shootRequest || revRequest) {
            //set target speeds
            shooterFTarget = -Constants.ShooterCal.interpolate(shooterTarget, true);
            shooterBTarget = -Constants.ShooterCal.interpolate(shooterTarget, false);

            //Check if Shooter Speed is within target range
            if ((ShooterFspeed > (shooterFTarget - Constants.Shooter_Speed_Tolerance))
                    && (ShooterFspeed < (shooterFTarget + Constants.Shooter_Speed_Tolerance))
                    && (ShooterBspeed > (shooterBTarget - Constants.Shooter_Speed_Tolerance))
                    && (ShooterBspeed < (shooterBTarget + Constants.Shooter_Speed_Tolerance))
                    || (runtime - shooterTimer > Constants.shooterMinTimeAtSpeed)
                    && (runtime - shooterTimer < (Constants.shooterMinTimeAtSpeed + Constants.shooterMinRunTime))
            ) {
                if (shooterTimer == 0) {
                    shooterTimer = runtime;
                    prevShotCount = shotCount;
                }
                if (runtime - shooterTimer > Constants.shooterMinTimeAtSpeed){
                    if (shootRequest){
                        shooterState = 1;
                        intake.setPower(Constants.Intake_Shoot_Speed);
                        Passthrough.setPower(1);
                        shotCount = prevShotCount + 1;
                    } else{
                        shooterState = 2;
                    }
                } else{
                    shooterState = 3;
                }
            } else {
                shooterTimer = 0;
                shooterState = 4;
            }
        } else {
            shooterFTarget = 0;
            shooterBTarget = 0;
            shooterState = 5;
        }

        shooterFMotor.setVelocity(shooterFTarget);
        shooterBMotor.setVelocity(shooterBTarget);

        if (isAuto && shooterState != 1 && shooterState != 5){
            intake.setPower(0);
            Passthrough.setPower(0);
        }

    }
}
