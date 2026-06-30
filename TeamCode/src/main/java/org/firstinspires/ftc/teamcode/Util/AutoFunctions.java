package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import static com.qualcomm.robotcore.util.TypeConversion.byteArrayToInt;

public class AutoFunctions
{

  public static double shooterTimer;
  public static int shooterState;
  public static int shotCount;
  private static int prevShotCount;

  public static double shooterFTarget;
  public static double shooterBTarget;

  public static boolean isAuto;


  public enum DetectedColour{
    GREEN,
    PURPLE,
    UNKNOWN,
  } 
  
  
  public static  DetectedColour getDetectedColor(Telemetry telemetry,
                                                         NormalizedColorSensor colourSensorA,
                                                         NormalizedColorSensor colourSensorB)
  {
    NormalizedRGBA colors1 = colourSensorA.getNormalizedColors(); // returns Red, Green, Blue, and Alpha
    NormalizedRGBA colors2 = colourSensorB.getNormalizedColors();

    float normRed1, normBlue1, normGreen1, normRed2, normBlue2, normGreen2, AverageSpinRed, AverageSpinBlue, AverageSpinGreen;
    normRed1 = colors1.red / colors1.alpha;
    normGreen1 = colors1.blue / colors1.alpha;
    normBlue1 = colors1.green / colors1.alpha;
    normRed2 = colors2.red / colors2.alpha;
    normBlue2 = colors2.blue / colors2.alpha;
    normGreen2 = colors2.green / colors2.alpha;

    AverageSpinRed = (normRed1 + normRed2) / 2;
    AverageSpinBlue = (normBlue1 + normBlue2) / 2;
    AverageSpinGreen = (normGreen1 + normGreen2) / 2;


    telemetry.addData("AverageSpinRed", (normRed1 + normRed2) / 2);
    telemetry.addData("AverageSpinBlue", (normBlue1 + normBlue2) / 2);
    telemetry.addData("AverageSpinGreen", (normGreen1 + normGreen2) / 2);

    if ((AverageSpinRed > 0.002&& AverageSpinRed < 0.0039) && (AverageSpinBlue > 0.0109 && AverageSpinBlue < 0.0117) && (AverageSpinGreen < 0.012 && AverageSpinGreen > 0.0093)) {
      telemetry.addData("Colour","green");
      return  DetectedColour.GREEN;
    }

    if ((AverageSpinRed > 0.0041 && AverageSpinRed < 0.0064) && (AverageSpinBlue > 0.0010 && AverageSpinBlue < 0.004) && (AverageSpinGreen > 0.0082 && AverageSpinGreen < 0.011)) {
      telemetry.addData("Colour","purple");
      return  DetectedColour.PURPLE;
    }

    return  DetectedColour.UNKNOWN;
  }

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

    if (isAuto && shooterState != 1){
      intake.setPower(0);
      Passthrough.setPower(0);
    }

  }
}
