package org.firstinspires.ftc.teamcode.Teleop;

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

  {
    pinpoint.setPosition(RobotPosition);
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

    {
      Pose2D robotPose = pinpoint.getPosition();
      telemetry.addData("position of robot", robotPose);
    }

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
    boolean spinToggleLast = false;
    boolean inToggleLast = false;
    boolean outToggleLast = false;
    boolean shootSequence = false;
    int shooterStage = 0;

    pinpoint.setOffsets(100, -25, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
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

      double rawForward = -gamepad1.left_stick_y; // FTC joystick forward is negative
      double rawStrafe = gamepad1.left_stick_x;
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

      //pinpoint.update(GoBildaPinpointDriver.ReadData.ONLY_UPDATE_HEADING);
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

      if (!shootSequence && gamepad2.a)
      {
        SpindxerServo.setPower(1);
      }
      else
      {
        SpindxerServo.setPower(0);
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


      if (shootSequence)
      {
        if (shooterStage == 1)
        {
          Flap.setPosition(0);
          shooterToggle = true;

        if (Shooter.getPower() == 1) {
        shooterStage = 2;
        }
        }
        if (shooterStage == 2)
        {
          Flap.setPosition(1);
          if (Shooter.getPower() < 1)
          {
            shooterStage = 3;
          }
        }
        if (shooterStage == 3)
        {
          Flap.setPosition(0);
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
        Flap.setPosition(0);
      }

      if (spindexerToggle) {
        SpindxerServo.setPower(Constants.spindexerPower);
      } else {
        SpindxerServo.setPower(0);
      }

      if (shooterToggle) {
        Shooter.setPower(Constants.shooterPower);
      } else {
        Shooter.setPower(0);
      }


      if (intakeToggle) {
        Intake.setPower(1);
      } else {
        Intake.setPower(0);

      }

      telemetry.update();
    }
  }
}

// stupid code wont commit without an adition
// THE CURRENT VERSION OF CODE DOES NOT WORK! MAKE SURE TO REMOVE THIS COMMENT AFTER
