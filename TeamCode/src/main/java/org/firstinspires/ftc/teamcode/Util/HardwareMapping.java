package org.firstinspires.ftc.teamcode.Util;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotor;
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
import org.firstinspires.ftc.teamcode.Util.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import static com.qualcomm.robotcore.util.TypeConversion.byteArrayToInt;
import org.firstinspires.ftc.teamcode.Util.Constants;

public class HardwareMapping
{

  // Hubs
  public Blinker control_Hub = hardwareMap.get(Blinker.class, "control_Hub");
  public  Blinker expansion_Hub_2 = hardwareMap.get(Blinker.class, "expansion_Hub_2");

  // Drive Motors
  public  DcMotor bLDrive = hardwareMap.get(DcMotor.class, "BLDrive");
  public  DcMotor bRDrive = hardwareMap.get(DcMotor.class, "BRDrive");
  public  DcMotor fLDrive = hardwareMap.get(DcMotor.class, "FLDrive");
  public  DcMotor fRDrive = hardwareMap.get(DcMotor.class, "FRDrive");

  // Mechanism Motors
  public  DcMotor Intake  = hardwareMap.get(DcMotor.class, "Intake");
  public  DcMotor Shooter = hardwareMap.get(DcMotor.class, "Shooter");

  // Internal Motion Units
  public  IMU imu                        = hardwareMap.get(IMU.class,                   "imu");
  public  GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

  // Servos
  public  CRServo IntakeTransferServo1 = hardwareMap.get(CRServo.class, "ITServo_1");
  public  CRServo IntakeTransferServo2 = hardwareMap.get(CRServo.class, "ITServo_2");
  public  CRServo SpindxerServo        = hardwareMap.get(CRServo.class, "Spindexer_Servo");
  public  Servo Flap                   = hardwareMap.get(Servo.class,   "Flap");

  // Colour Sensors
  public NormalizedColorSensor SpindexerSensor1 = hardwareMap.get(NormalizedColorSensor.class, "spindexer_colour_1");
  public NormalizedColorSensor SpindexerSensor2 = hardwareMap.get(NormalizedColorSensor.class, "spindexer_colour_2");

  //Getter-Setters
  public NormalizedColorSensor getSpindexerSensor2() {
    return SpindexerSensor2;
  }

  public void setSpindexerSensor2(NormalizedColorSensor spindexerSensor2) {
    SpindexerSensor2 = spindexerSensor2;
  }

  public Servo getFlap() {
    return Flap;
  }

  public void setFlap(Servo flap) {
    Flap = flap;
  }
}
