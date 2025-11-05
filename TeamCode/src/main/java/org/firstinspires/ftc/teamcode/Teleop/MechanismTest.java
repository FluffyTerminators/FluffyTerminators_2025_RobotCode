package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Util.Constants;
import org.firstinspires.ftc.teamcode.Util.GoBildaPinpointDriver;
//import static org.firstinspires.ftc.teamcode.Util.Constants.HardwareMappings.*;


//Download Missing Files


@TeleOp(name = "MechanismTest")
public class MechanismTest extends LinearOpMode {

  // Hubs
  //public Blinker control_Hub;
  //public Blinker expansion_Hub_2;

  // Drive Motors
  public DcMotor bLDrive;
  public DcMotor bRDrive;
  public DcMotor fLDrive;
  public DcMotor fRDrive;

  // Mechanism Motors
  public DcMotor Intake;
  public DcMotorEx Shooter;

  // Internal Motion Units
  public IMU imu;
  public GoBildaPinpointDriver pinpoint;

  // Servos
  //public CRServo IntakeTransferServo1 = hardwareMap.get(CRServo.class, "ITServo_1");
  //public CRServo IntakeTransferServo2 = hardwareMap.get(CRServo.class, "ITServo_2");
  public CRServo SpindxerServo;
  public Servo Flap;

  // Colour Sensors
  public NormalizedColorSensor SpindexerSensor1;
  public NormalizedColorSensor SpindexerSensor2;

  public enum DetectedColour {
    GREEN,
    PURPLE,
    UNKNOWN,
  }

  public DetectedColour getDetectedColor(Telemetry telemetry) {
    NormalizedRGBA colors1 = SpindexerSensor1.getNormalizedColors(); // returns Red, Green, Blue, and Alpha
    NormalizedRGBA colors2 = SpindexerSensor2.getNormalizedColors();

    float normRed1, normBlue1, normGreen1, normRed2, normBlue2, normGreen2;
    normRed1 = colors1.red / colors1.alpha;
    normGreen1 = colors1.blue / colors1.alpha;
    normBlue1 = colors1.green / colors1.alpha;
    normRed2 = colors2.red / colors1.alpha;
    normBlue2 = colors2.blue / colors1.alpha;
    normGreen2 = colors2.green / colors1.alpha;

    telemetry.addData("AverageSpinRed", (normRed1 + normRed2) / 2);
    telemetry.addData("AverageSpinBlue", (normBlue1 + normBlue2) / 2);
    telemetry.addData("AverageSpinGreen", (normGreen1 + normGreen2) / 2);

    return DetectedColour.UNKNOWN;
  }

  public void runOpMode() throws InterruptedException {
    //control_Hub = hardwareMap.get(Blinker.class, "control_Hub");
    //expansion_Hub_2 = hardwareMap.get(Blinker.class, "expansion_Hub_2");
    bLDrive = hardwareMap.get(DcMotor.class, "BLDrive");
    bRDrive = hardwareMap.get(DcMotor.class, "BRDrive");
    fLDrive = hardwareMap.get(DcMotor.class, "FLDrive");
    fRDrive = hardwareMap.get(DcMotor.class, "FRDrive");
    Intake  = hardwareMap.get(DcMotor.class, "Intake");
    Shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
    imu = hardwareMap.get(IMU.class,  "imu");
    pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
    SpindxerServo = hardwareMap.get(CRServo.class, "Spindexer_Servo");
    Flap = hardwareMap.get(Servo.class,   "Spindexer_Flap_Servo");
    SpindexerSensor1 = hardwareMap.get(NormalizedColorSensor.class, "spindexer_colour_1");
    SpindexerSensor2 = hardwareMap.get(NormalizedColorSensor.class, "spindexer_colour_2");

    fLDrive.setDirection(DcMotorSimple.Direction.REVERSE);
    bLDrive.setDirection(DcMotorSimple.Direction.REVERSE);
    fRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    bRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    fLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    bLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    imu.initialize(new IMU.Parameters((ImuOrientationOnRobot) new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.DOWN, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT)));

    double Forward = 0;
    double Strafe = 0;
    double Turn = 0;
    double MotorPower = 0;
    double Heading = 0;
    boolean intakeToggle = false;
    boolean shooterToggle = false;
    boolean spindexerToggle = true;
    boolean spinToggleLast = false;
    boolean inToggleLast = false;
    boolean outToggleLast = false;
    boolean shootSequence = false;
    int shooterStage = 0;
    int ShooterCurrent,ShooterLast = 0;
    long TimeCurrent,TimeLast = 0;
    double Shooterspeed = 0;
    double flapPos = 0.50;
    double ShooterTarget = 0;
    double ShooterPower = 0;

    pinpoint.setOffsets(0, 0, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
    pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
    pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

    telemetry.addData("Status", "Initialized");
    telemetry.update();
    waitForStart();
    while (opModeIsActive()) {
      telemetry.addData("Status", "Running");
      pinpoint.update();
      telemetry.addData("Heading Scalar", pinpoint.getYawScalar());
      Heading = Math.toRadians(pinpoint.getPosition().getHeading(AngleUnit.DEGREES) + Constants.HeadingOffset);
      telemetry.addData("Heading", Math.toDegrees(Heading));
      ShooterCurrent = Shooter.getCurrentPosition();
      TimeCurrent = System.currentTimeMillis();
      double CalcShooterspeed = (double)(ShooterCurrent - ShooterLast)/(double)(TimeCurrent-TimeLast);
      ShooterLast = ShooterCurrent;
      TimeLast = TimeCurrent;
      Shooterspeed = Shooter.getVelocity();
      if (gamepad1.y)
      {
        fRDrive.setPower(gamepad1.left_stick_y);
        telemetry.addData("Active Motor: ","Front Right");
      }
      else
      {
        fRDrive.setPower(0);
      }
      if (gamepad1.x)
      {
        fLDrive.setPower(gamepad1.left_stick_y);
        telemetry.addData("Active Motor: ","Front Left");
      }
      else
      {
        fLDrive.setPower(0);
      }
      if (gamepad1.a)
      {
        bRDrive.setPower(gamepad1.left_stick_y);
        telemetry.addData("Active Motor: ","Back Right");
      }
      else
      {
        bRDrive.setPower(0);
      }
      if (gamepad1.b)
      {
        bLDrive.setPower(gamepad1.left_stick_y);
        telemetry.addData("Active Motor: ","Back Left");
      }
      else
      {
        bLDrive.setPower(0);
      }

      if (gamepad1.dpad_up)
      {
        ShooterTarget += 10;
      }

      if (gamepad1.dpad_down)
      {
        ShooterTarget -= 10;
      }
      telemetry.addData("ShooterTarget", ShooterTarget);

      if (gamepad1.left_bumper)
      {
        if (Shooterspeed < ShooterTarget)
        {
          ShooterPower += 0.01;
        }

        if (Shooterspeed > ShooterTarget)
        {
          ShooterPower -= 0.01;
        }

        Shooter.setVelocity(ShooterTarget);

        telemetry.addData("Active Motor: ","Shooter");
      }
      else
      {
        Shooter.setPower(0);
      }
      if (gamepad1.right_bumper)
      {
        flapPos = gamepad1.left_stick_y;
      }
      if (gamepad1.dpad_up)
      {
        flapPos = 0.55;
      }
      if (gamepad1.dpad_down)
      {
        flapPos = 0.45;
      }
      if (gamepad1.dpad_left)
        flapPos = 0.50;
      Flap.setPosition(flapPos);
      telemetry.addData("Front Right Encoder: ",fRDrive.getCurrentPosition());
      telemetry.addData("Front Left Encoder: ",fLDrive.getCurrentPosition());
      telemetry.addData("Back Right Encoder: ",bRDrive.getCurrentPosition());
      telemetry.addData("Back Left Encoder: ",bLDrive.getCurrentPosition());
      telemetry.addData("Flap Servo Set Position: ",flapPos);
      telemetry.addData("Flap Servo reported position: ", Flap.getPosition());
      telemetry.addData("Shooter Encoder: ",ShooterCurrent);
      telemetry.addData("Shooter Speed (ticks/sec): ",Shooterspeed);
      telemetry.addData("Calculated Shooter Speed (ticks/sec): ",(CalcShooterspeed * 1000));
      telemetry.addData("ShooterPower", ShooterPower);
      telemetry.update();
    }
  }
}

