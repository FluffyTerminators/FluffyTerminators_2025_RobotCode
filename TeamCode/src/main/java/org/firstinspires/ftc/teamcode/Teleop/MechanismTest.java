package org.firstinspires.ftc.teamcode.Teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Util.Constants;
import org.firstinspires.ftc.teamcode.Util.GoBildaPinpointDriver;
//import static org.firstinspires.ftc.teamcode.Util.Constants.HardwareMappings.*;


//Download Missing Files


@TeleOp(name = "MechanismTest")
@Configurable // Panels
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
  public DcMotorEx ShooterFront;
  public DcMotorEx ShooterBack;

  // Internal Motion Units
  public IMU imu;
  public GoBildaPinpointDriver pinpoint;

  // Servos
  //public CRServo IntakeTransferServo1 = hardwareMap.get(CRServo.class, "ITServo_1");
  //public CRServo IntakeTransferServo2 = hardwareMap.get(CRServo.class, "ITServo_2");
  public CRServo SpindxerServo;

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

    float normRed1, normBlue1, normGreen1, normRed2, normBlue2, normGreen2, AverageSpinRed, AverageSpinBlue, AverageSpinGreen;
    normRed1 = colors1.red / colors1.alpha;
    normGreen1 = colors1.blue / colors1.alpha;
    normBlue1 = colors1.green / colors1.alpha;
    normRed2 = colors2.red / colors2.alpha;
    normBlue2 = colors2.blue / colors2.alpha;
    normGreen2 = colors2.green / colors2.alpha;

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
    ShooterFront = hardwareMap.get(DcMotorEx.class, "ShooterFront");
    ShooterBack = hardwareMap.get(DcMotorEx.class, "ShooterBack");
    imu = hardwareMap.get(IMU.class,  "imu");
    pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
    SpindxerServo = hardwareMap.get(CRServo.class, "Spindexer_Servo");
    SpindexerSensor1 = hardwareMap.get(NormalizedColorSensor.class, "spindexer_colour_1");
    SpindexerSensor2 = hardwareMap.get(NormalizedColorSensor.class, "spindexer_colour_2");

    fLDrive.setDirection(DcMotorSimple.Direction.REVERSE);
    bLDrive.setDirection(DcMotorSimple.Direction.REVERSE);
    fRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    bRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    fLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    bLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    ShooterBack.setDirection(DcMotorSimple.Direction.REVERSE);
    ShooterFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    ShooterBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
    double ShooterFspeed = 0;
    double ShooterBspeed = 0;
    double P = 0;
    double I = 0;
    double D = 0;
    boolean aLast = false;
    boolean bLast = false;
    boolean xLast = false;
    boolean yLast = false;
    boolean upLast = false;
    boolean downLast = false;

    pinpoint.setOffsets(0, 0, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
    pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
    pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
    //PIDFCoefficients Motorsettings = ShooterFront.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
    P = Constants.PID_P;
    I = Constants.PID_I;
    D = Constants.PID_D;
    telemetry.addData("Status", "Initialized");
    telemetry.update();
    waitForStart();
    while (opModeIsActive()) {
      telemetry.addData("Status", "Running");
      pinpoint.update();
      telemetry.addData("Heading Scalar", pinpoint.getYawScalar());
      Heading = Math.toRadians(pinpoint.getPosition().getHeading(AngleUnit.DEGREES) + Constants.HeadingOffset);
      telemetry.addData("Heading", Math.toDegrees(Heading));
     // ShooterCurrent = ShooterFront.getCurrentPosition();
     // TimeCurrent = System.currentTimeMillis();
     // double CalcShooterspeed = (double)(ShooterCurrent - ShooterLast)/(double)(TimeCurrent-TimeLast);
     // ShooterLast = ShooterCurrent;
     // TimeLast = TimeCurrent;
      ShooterFspeed = ShooterFront.getVelocity();
      ShooterBspeed = ShooterBack.getVelocity();

      Intake.setPower(gamepad1.left_trigger);
      SpindxerServo.setPower(gamepad1.right_trigger);
      if (gamepad1.dpad_up)
      {
        if (!upLast) {
          ShooterTarget += 10;
        }
        upLast = true;
      } else {
        upLast = false;
      }

      if (gamepad1.dpad_down)
      {
        if (!downLast) {
          ShooterTarget -= 10;
        }
        downLast = true;
      } else {
        downLast = false;
      }
      telemetry.addData("ShooterTarget", ShooterTarget);

      if (gamepad1.left_bumper)
      {
        ShooterFront.setVelocity(ShooterTarget);
        ShooterBack.setVelocity(ShooterTarget);
        telemetry.addData("Active Motor: ","Shooter Front");
      }
      else
      {
        ShooterFront.setPower(0);
        ShooterBack.setPower(0);
      }

      if (gamepad1.x)
      {
        if (!xLast) {
          P = P + 0.1;
        }
        xLast = true;
      } else {
        xLast = false;
      }
      if (gamepad1.a)
      {
        if (!aLast) {
          I = I + 0.1;
        }
        aLast = true;
      } else {
        aLast = false;
      }
      if (gamepad1.b)
      {
        if (!bLast) {
          D = D + 0.1;
        }
        bLast = true;
      } else {
        bLast = false;
      }
      if (gamepad1.y)
      {
        ShooterBack.setVelocityPIDFCoefficients(P, I, D, 0);
        ShooterFront.setVelocityPIDFCoefficients(P, I, D, 0);
      }

      telemetry.addData("PIDF Values Back", ShooterBack.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
      telemetry.addData("PIDF Values Front", ShooterFront.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
      telemetry.addData("P", P);
      telemetry.addData("I", I);
      telemetry.addData("D", D);
      /*telemetry.addData("Front Right Encoder: ",fRDrive.getCurrentPosition());
      telemetry.addData("Front Left Encoder: ",fLDrive.getCurrentPosition());
      telemetry.addData("Back Right Encoder: ",bRDrive.getCurrentPosition());
      telemetry.addData("Back Left Encoder: ",bLDrive.getCurrentPosition());
      telemetry.addData("Flap Servo Set Position: ",flapPos);*/
      telemetry.addData("Shooter Front (ticks/sec): ",ShooterFspeed);
      telemetry.addData("Shooter Back (ticks/sec): ",ShooterBspeed);
      telemetry.addData("ShooterPower", ShooterPower);
      telemetry.update();
    }
  }
}

