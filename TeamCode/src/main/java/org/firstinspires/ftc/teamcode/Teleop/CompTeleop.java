package org.firstinspires.ftc.teamcode.Teleop;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Util.GoBildaPinpointDriver;
import static com.qualcomm.robotcore.util.TypeConversion.byteArrayToInt;
import static org.firstinspires.ftc.teamcode.Util.Constants.*;

import org.firstinspires.ftc.teamcode.Util.Constants;


//Download Missing Files


@TeleOp(name = "CompTeleOp")
public class CompTeleop extends LinearOpMode {

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
  public DcMotor Shooter;

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
  public Pose2D RobotPosition = new Pose2D(DistanceUnit.CM, 0, 0,AngleUnit.DEGREES,0 );
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
    Shooter = hardwareMap.get(DcMotor.class, "Shooter");
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
    double spindexerPower = spindexerFWD;
    boolean spinToggleLast = false;
    boolean inToggleLast = false;
    boolean outToggleLast = false;
    boolean shootSequence = false;
    int shooterStage = 0;
    double lastRuntime = getRuntime();
    int S_lastencoder = 0;
    int S_encoder = 0;
    double S_lastime = 0;
    double S_time = 0;
    double S_Targetspeed = 0;
    double S_lastMotorpower = 0;
    double S_lastSpeed = 0;
    boolean Last2DU = false;
    boolean Last2DL = false;
    boolean Last2DD = false;


    pinpoint.setOffsets(100, -25, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
    pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
    pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
    Flap.setPosition(flapUp);
    telemetry.addData("Status", "Initialized");
    telemetry.update();
    waitForStart();
    while (opModeIsActive()) {
      telemetry.addData("Status", "Running");
      pinpoint.update();
      telemetry.addData("Heading Scalar", pinpoint.getYawScalar());
      Heading = Math.toRadians(pinpoint.getPosition().getHeading(AngleUnit.DEGREES) + Constants.HeadingOffset);
      telemetry.addData("Heading", Math.toDegrees(Heading));

      double rawForward = gamepad1.left_stick_y; // FTC joystick forward is negative
      double rawStrafe = -gamepad1.left_stick_x;
      Turn = -gamepad1.right_stick_x;

      double sinHeading = Math.sin(Heading);
      double cosHeading = Math.cos(Heading);

      // Rotate the driver input vector so it is field-centric
      Strafe = rawStrafe * cosHeading - rawForward * sinHeading;
      Forward = rawStrafe * sinHeading + rawForward * cosHeading;

      if (gamepad1.right_bumper)
      {
        Forward /= Constants.brake;
        Strafe /= Constants.brake;
        Turn /= Constants.brake;
      }

      if (gamepad1.left_bumper)
      {
        imu.initialize(new IMU.Parameters((ImuOrientationOnRobot) new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT)));
        imu.resetYaw();
        pinpoint.resetPosAndIMU(); //resets the position to 0 and recalibrates the IMU
        pinpoint.setHeading(0, AngleUnit.DEGREES);
        pinpoint.update();
      }

      telemetry.addData("PinPoint Status", pinpoint.getDeviceStatus());
      telemetry.addData("forward",Forward);
      telemetry.addData("strafe",Strafe);
      telemetry.addData("Turn", Turn);

      double denominator = Math.max(Math.abs(Forward) + Math.abs(Strafe) + Math.abs(Turn), 1);
      double frontLeftPower = (Forward + Strafe + Turn) / denominator;
      double backLeftPower = (Forward - Strafe + Turn) / denominator;
      double frontRightPower = (Forward - Strafe - Turn) / denominator;
      double backRightPower = (Forward + Strafe - Turn) / denominator;


      fLDrive.setPower(frontLeftPower);
      telemetry.addData("FLDrive",frontLeftPower);
      bLDrive.setPower(backLeftPower);
      telemetry.addData("BLDrive",backLeftPower);
      fRDrive.setPower(frontRightPower);
      telemetry.addData("FRDrive",frontRightPower);
      bRDrive.setPower(backRightPower);
      telemetry.addData("BRDrive",backRightPower);

      telemetry.addData("FRDrive_Actual", fRDrive.getPower());
      telemetry.addData("FLDrive_Actual", fLDrive.getPower());
      telemetry.addData("BRDrive_Actual", bRDrive.getPower());
      telemetry.addData("BLDrive_Actual", bLDrive.getPower());


      DetectedColour Colour = getDetectedColor(telemetry);

      if (gamepad1.b) {
        pinpoint.recalibrateIMU(); //recalibrates the IMU without resetting position
      }

      if (gamepad2.left_bumper) {
        if (shootSequence)
        {
          shootSequence = false;
        }
        else {
          shootSequence = true;
          shooterStage = 1; // 1 - spinning up/deploy , 2 - load artifact , 3 - fire , 4 - spin down/park
        }
      }

      if (gamepad2.b)
      {
        spindexerPower = spindexerBWD;
      } else
      {
        spindexerPower = spindexerFWD;
      }

      if (gamepad2.x)
      {
        if (!spinToggleLast)
        {
          spindexerToggle = !spindexerToggle;
          spinToggleLast= true;
        }
      } else
      {
        spinToggleLast = false;
      }


      if (gamepad2.right_bumper)
      {
        if (!inToggleLast)
        {
          intakeToggle = !intakeToggle;
          inToggleLast= true;
        }
      } else
      {
        inToggleLast = false;
      }

      if (gamepad2.left_bumper)
      {
        Flap.setPosition(flapDeploy);
      } else
      {
        Flap.setPosition(flapUp);
      }


     /* if (shootSequence)
      {
        lastRuntime = getRuntime();
        if (shooterStage == 1)
        {
          Flap.setPosition(flapDeploy);
          if (getRuntime() == (lastRuntime + 400)) {
            shooterToggle = true;
          }
        if (Shooter.getPower() == 1) {
          lastRuntime = getRuntime();
          if (getRuntime() == (lastRuntime + 400)) {
            shooterToggle = true;
          }
        shooterStage = 2;
        }
        }
        if (shooterStage == 2)
        {
          Flap.setPosition(flapUp);
          lastRuntime = getRuntime();
          if (getRuntime() == (lastRuntime + 400)) {
            shooterToggle = true;
          }
          if (Shooter.getPower() < 1)
          {
            shooterStage = 3;
          }
        }
        if (shooterStage == 3)
        {
          Flap.setPosition(flapUp);
          shooterStage = 4;
        }
        if (shooterStage == 4)
        {
          shooterToggle = false;
        }
        shootSequence = false;
      }
      else
      {
        shooterToggle = false;
        Flap.setPosition(flapUp);
      }b */

      if (spindexerToggle) {
        SpindxerServo.setPower(spindexerPower);
      } else {
        SpindxerServo.setPower(0);
      }

      /*if (shooterToggle) {
        Shooter.setPower(Constants.shooterPower);
      } else {
        Shooter.setPower(0);
      }*/
      S_time = getRuntime();
      S_encoder = Shooter.getCurrentPosition();
      double S_Speed = (S_encoder - S_lastencoder) / (S_time - S_lastime);
      S_lastencoder = S_encoder;
      S_lastime = S_time;
      telemetry.addData("Shooter Target Speed",S_Targetspeed);
      telemetry.addData("Shooter Speed (tick/sec)",S_Speed);
      telemetry.addData("Shooter speed (RPM)",(S_Speed / 60) / 21);
      double S_motorpower = 0;
      if (shooterToggle) {
        S_motorpower = gamepad2.left_trigger;
        Shooter.setPower(S_motorpower);
        telemetry.addData("Shooter Mode","Manual");
        telemetry.addData("Shooter Power", S_motorpower);
      }
      else
      {
        S_motorpower = Shooter.getPower();
        if ((S_Speed < S_Targetspeed - 10) && (S_Speed - S_lastSpeed < 10) && (S_motorpower - S_lastMotorpower < 0.01))
        {
          S_lastMotorpower = S_motorpower;
          S_motorpower += 0.01;
        }
        else if ((S_Speed > S_Targetspeed +10) && (S_Speed - S_lastSpeed > -10) && (S_motorpower - S_lastMotorpower > -0.01))
        {
          S_lastMotorpower = S_motorpower;
          S_motorpower -= 0.01;
        }
        else
        {
          S_lastMotorpower = S_motorpower;
        }
        if (S_motorpower < -1) {S_motorpower = -1;}
        if (S_motorpower > 1) {S_motorpower = 1;}
        Shooter.setPower(S_motorpower);
        telemetry.addData("Shooter Mode","Speed Control");
        telemetry.addData("Shooter Power", S_motorpower);
      }
      if (gamepad2.dpad_left)
      {
        if (!(Last2DL)) {
          shooterToggle = !shooterToggle;
        }
        Last2DL = true;
      }
      else
      {
        Last2DL = false;
      }
      if (gamepad2.dpad_up)
      {
        if (!(Last2DU)) {
          S_Targetspeed += 100;
          if (S_Targetspeed > 2000) {
            S_Targetspeed = 2000;
          }
        }
        Last2DU = true;
      }
      else
      {
        Last2DU = false;
      }
      if (gamepad2.dpad_down)
      {
        if (!(Last2DD)) {
          S_Targetspeed -= 100;
          if (S_Targetspeed < 0) {
            S_Targetspeed = 0;
          }
        }
        Last2DD = true;
      }


      if (intakeToggle) {
        Intake.setPower(0.75);
      } else {
        Intake.setPower(0);

      }

      telemetry.update();
    }
  }
}
