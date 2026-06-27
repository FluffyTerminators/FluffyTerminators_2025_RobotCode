package org.firstinspires.ftc.teamcode.Util;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.GoBildaOdometryPods;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import static com.qualcomm.robotcore.util.TypeConversion.byteArrayToInt;

import static java.sql.Types.NULL;

import org.firstinspires.ftc.teamcode.Util.Constants.*;

import org.firstinspires.ftc.teamcode.Teleop.LimeComp;
import org.firstinspires.ftc.teamcode.Util.Constants;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.GoBildaOdometryPods.*;

public class AutoFunctions
{

  public static double shooterTimer;
  public static int shooterState;
  public static int shotCount;
  private static int prevShotCount;


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

  public static double runShooter(DcMotorEx shooterFMotor,
                                DcMotorEx shooterBMotor,
                                DcMotorEx intake,
                                boolean passthroughToggle,
                                boolean intakeToggle,
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
      shooterFMotor.setVelocity(0);
      shooterBMotor.setVelocity(0);
      shooterState = 0;
    } else if (shootRequest || revRequest) {
      //set target speeds
      double ShooterFTarget = -Constants.ShooterCal.interpolate(shooterTarget, true);
      double ShooterBTarget = -Constants.ShooterCal.interpolate(shooterTarget, false);
      shooterFMotor.setVelocity(ShooterFTarget);
      shooterBMotor.setVelocity(ShooterBTarget);

      //Check if Shooter Speed is within target range
      if ((ShooterFspeed > (ShooterFTarget - Constants.Shooter_Speed_Tolerance))
              && (ShooterFspeed < (ShooterFTarget + Constants.Shooter_Speed_Tolerance))
              && (ShooterBspeed > (ShooterBTarget - Constants.Shooter_Speed_Tolerance))
              && (ShooterBspeed < (ShooterBTarget + Constants.Shooter_Speed_Tolerance))
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
      shooterFMotor.setVelocity(0);
      shooterBMotor.setVelocity(0);
      shooterState = 5;
    }

    if (shotCount == 5)
    {
      shotCount = 0;
    }
    return shotCount;
  }
}
