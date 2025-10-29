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
  private Blinker control_Hub = hardwareMap.get(Blinker.class, "control_Hub");
  private  Blinker expansion_Hub_2 = hardwareMap.get(Blinker.class, "expansion_Hub_2");

  // Drive Motors
  private  DcMotor bLDrive = hardwareMap.get(DcMotor.class, "BLDrive");
  private  DcMotor bRDrive = hardwareMap.get(DcMotor.class, "BRDrive");
  private  DcMotor fLDrive = hardwareMap.get(DcMotor.class, "FLDrive");
  private  DcMotor fRDrive = hardwareMap.get(DcMotor.class, "FRDrive");

  // Mechanism Motors
  private  DcMotor Intake  = hardwareMap.get(DcMotor.class, "Intake");
  private  DcMotor Shooter = hardwareMap.get(DcMotor.class, "Shooter");

  // Internal Motion Units
  private  IMU imu                        = hardwareMap.get(IMU.class,                   "imu");
  private  GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

  // Servos
  private  CRServo IntakeTransferServo1 = hardwareMap.get(CRServo.class, "ITServo_1");
  private  CRServo IntakeTransferServo2 = hardwareMap.get(CRServo.class, "ITServo_2");
  private  CRServo SpindxerServo        = hardwareMap.get(CRServo.class, "Spindexer_Servo");
  private  Servo Flap                   = hardwareMap.get(Servo.class,   "Flap");

  // Colour Sensors
  private NormalizedColorSensor SpindexerSensor1 = hardwareMap.get(NormalizedColorSensor.class, "spindexer_colour_1");
  private NormalizedColorSensor SpindexerSensor2 = hardwareMap.get(NormalizedColorSensor.class, "spindexer_colour_2");

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
