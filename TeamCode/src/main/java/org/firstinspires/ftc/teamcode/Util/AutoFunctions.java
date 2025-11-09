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
import org.firstinspires.ftc.teamcode.Util.Constants.*;

import org.firstinspires.ftc.teamcode.Teleop.LimeComp;
import org.firstinspires.ftc.teamcode.Util.Constants;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.GoBildaOdometryPods.*;

public class AutoFunctions
{
  public static LimeComp.DetectedColour getDetectedColor(Telemetry telemetry,
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
      return LimeComp.DetectedColour.GREEN;
    }

    if ((AverageSpinRed > 0.0041 && AverageSpinRed < 0.0064) && (AverageSpinBlue > 0.0010 && AverageSpinBlue < 0.004) && (AverageSpinGreen > 0.0082 && AverageSpinGreen < 0.011)) {
      telemetry.addData("Colour","purple");
      return LimeComp.DetectedColour.PURPLE;
    }

    return LimeComp.DetectedColour.UNKNOWN;
  }

  public static class ShooterController
  {
    private static final double FEED_DELAY_SECONDS = 0.25;

    private boolean spindexToggle;
    private double lastRunTime;
    private int shooterState;

    public ShooterController()
    {
      reset();
    }

    public void reset()
    {
      spindexToggle = false;
      lastRunTime = 0;
      shooterState = 0;
    }

    public boolean isFinished()
    {
      return shooterState == -1;
    }

    public int getShooterState()
    {
      return shooterState;
    }

    public void runShooter(Telemetry telemetry,
                           DcMotorEx shooterMotor,
                           Servo flap,
                           CRServo spindexerServo,
                           NormalizedColorSensor colourSensorA,
                           NormalizedColorSensor colourSensorB,
                           double shooterTarget,
                           double runtimeSeconds)
    {
      shooterMotor.setVelocity(shooterTarget);

      telemetry.addData("Shooter Target", shooterTarget);
      telemetry.addData("Shooter State", shooterState);
      telemetry.addData("Spindexer Active", spindexToggle);

      switch (shooterState)
      {
        case 0: // Spin up and start feeding when ready
          spindexToggle = true;
          shooterState = 1;
          break;

        case 1: // Wait for a detected note before dropping the flap
          LimeComp.DetectedColour detectedColour = getDetectedColor(telemetry, colourSensorA, colourSensorB);
          telemetry.addData("Detected Colour", detectedColour);
          if (detectedColour == LimeComp.DetectedColour.GREEN || detectedColour == LimeComp.DetectedColour.PURPLE)
          {
            lastRunTime = runtimeSeconds;
            shooterState = 2;
          }
          break;

        case 2: // Drop flap to feed the note
          spindexToggle = false;
          flap.setPosition(Constants.flapDeploy);
          if (runtimeSeconds - lastRunTime > FEED_DELAY_SECONDS)
          {
            lastRunTime = runtimeSeconds;
            shooterState = 3;
          }
          break;

        case 3: // Kick spindexer to clear the chamber and wait for flywheel drop
          spindexToggle = true;
          if (runtimeSeconds - lastRunTime > FEED_DELAY_SECONDS)
          {
            spindexToggle = false;
            if (shooterMotor.getVelocity() < shooterTarget)
            {
              shooterState = 4;
            }
          }
          break;

        case 4: // Retract flap once velocity recovers, then finish cycle
          flap.setPosition(Constants.flapUp);
          if (shooterMotor.getVelocity() > shooterTarget)
          {
            shooterMotor.setVelocity(0);
            shooterState = -1;
          }
          break;

        case -1:
        default:
          spindexToggle = false;
          shooterMotor.setVelocity(0);
          break;
      }

      if (spindexerServo != null)
      {
        spindexerServo.setPower(spindexToggle ? 1 : 0);
      }
    }
  }
}
