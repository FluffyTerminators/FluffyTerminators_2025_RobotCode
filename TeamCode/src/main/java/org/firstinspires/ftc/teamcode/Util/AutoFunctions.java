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
import org.firstinspires.ftc.teamcode.Util.ShooterPidTuning;

import org.firstinspires.ftc.teamcode.Teleop.LimeComp;
import org.firstinspires.ftc.teamcode.Util.Constants;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.GoBildaOdometryPods.*;

public class AutoFunctions
{

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

  public static void runShooter(DcMotorEx shooterMotor,
                                Boolean spindexToggle,
                                double shooterVel,
                                double spinUpTime,
                                double shooterState,
                                NormalizedColorSensor colourSensorA,
                                NormalizedColorSensor colourSensorB,
                                Servo flap)
  {
    ShooterPidTuning.applyTo(shooterMotor);
    shooterMotor.setVelocity(shooterVel);
    telemetry.addData("Shooter Motor", shooterMotor);
    telemetry.addData("Spindexer?", spindexToggle);
    telemetry.addData("Shooter Target", shooterVel);
    telemetry.addData("Runtime", spinUpTime);
    telemetry.addData("Shooter State", shooterState);
    telemetry.addData("First Colour", colourSensorA);
    telemetry.addData("Second Colour", colourSensorB);
    telemetry.addData("Servo", flap);
    spinUpTime = linearOpMode.getRuntime();
    shooterState = 1;
    if (shooterState == 1)
    {
      spindexToggle = true;
      if (getDetectedColor(telemetry, colourSensorA, colourSensorB) ==  DetectedColour.GREEN || getDetectedColor(telemetry, colourSensorA, colourSensorB) ==  DetectedColour.PURPLE)
      {
        spinUpTime = linearOpMode.getRuntime();
        shooterState = 2;
      }
    }
    if (shooterState == 2)
    {
      flap.setPosition(Constants.flapDeploy);
      if (linearOpMode.getRuntime() == spinUpTime + 0.5)
      {
        shooterState = 3;
      }
    }
    if (shooterState == 3)
    {
      spindexToggle = false;
      if (shooterMotor.getVelocity() < shooterVel)
      {
        shooterState = 4;
      }
    }
    if (shooterState == 4)
    {
      flap.setPosition(Constants.flapUp);
      if (shooterMotor.getVelocity() > shooterVel)
      {
        shooterState = -1;
      }
    }
  }
}
