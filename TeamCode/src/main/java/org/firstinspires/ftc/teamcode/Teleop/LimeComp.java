package org.firstinspires.ftc.teamcode.Teleop;

import static org.firstinspires.ftc.teamcode.Util.Constants.flapDeploy;
import static org.firstinspires.ftc.teamcode.Util.Constants.flapUp;
import static org.firstinspires.ftc.teamcode.Util.Constants.spindexerBWD;
import static org.firstinspires.ftc.teamcode.Util.Constants.spindexerFWD;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Util.Constants;
import org.firstinspires.ftc.teamcode.Util.GoBildaPinpointDriver;
import java.util.List;
//Download Missing Files

@TeleOp(name = "LimelightComp")
public class LimeComp extends LinearOpMode {

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
  public CRServo SpindxerServo;
  public Servo Flap;

  // Colour Sensors
  public ColorRangeSensor SpindexerSensor1;
  public ColorRangeSensor SpindexerSensor2;
  public Pose2D RobotPosition = new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.DEGREES, 0);

  public Limelight3A limelight;

  public enum Distance {
    LOADED,
    EMPTY
  }

  private static final double OBJECT_DETECTION_RANGE_CM = 4.0;

  public Distance getDetectedColor(Telemetry telemetry) {
    double sensor1DistanceCm = SpindexerSensor1.getDistance(DistanceUnit.CM);
    double sensor2DistanceCm = SpindexerSensor2.getDistance(DistanceUnit.CM);
    double usableSensor1 = Double.isNaN(sensor1DistanceCm) ? Double.POSITIVE_INFINITY : sensor1DistanceCm;
    double usableSensor2 = Double.isNaN(sensor2DistanceCm) ? Double.POSITIVE_INFINITY : sensor2DistanceCm;
    double closestDistance = Math.min(usableSensor1, usableSensor2);

    telemetry.addData("SpindexerDist1(cm)", sensor1DistanceCm);
    telemetry.addData("SpindexerDist2(cm)", sensor2DistanceCm);
    telemetry.addData("SpindexerClosest(cm)", closestDistance);

    if (closestDistance <= OBJECT_DETECTION_RANGE_CM) {
      telemetry.addData("ObjectDetected", true);
      return Distance.LOADED;
    }

    telemetry.addData("ObjectDetected", false);
    return Distance.EMPTY;
  }

  public void runOpMode() throws InterruptedException {
    bLDrive = hardwareMap.get(DcMotor.class, "BLDrive");
    bRDrive = hardwareMap.get(DcMotor.class, "BRDrive");
    fLDrive = hardwareMap.get(DcMotor.class, "FLDrive");
    fRDrive = hardwareMap.get(DcMotor.class, "FRDrive");
    Intake = hardwareMap.get(DcMotor.class, "Intake");
    Shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
    imu = hardwareMap.get(IMU.class, "imu");
    pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
    SpindxerServo = hardwareMap.get(CRServo.class, "Spindexer_Servo");
    Flap = hardwareMap.get(Servo.class, "Spindexer_Flap_Servo");
    SpindexerSensor1 = hardwareMap.get(ColorRangeSensor.class, "spindexer_colour_1");
    SpindexerSensor2 = hardwareMap.get(ColorRangeSensor.class, "spindexer_colour_2");
    limelight = hardwareMap.get(Limelight3A.class, "Limelight");

    limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
    limelight.start(); // This tells Limelight to start looking!
    limelight.pipelineSwitch(0); // Switch to pipeline number 0

    LLResult result = limelight.getLatestResult();

    telemetry.addData("Current Pipeline = ", result.getPipelineIndex());

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
    double ShooterTarget = 0;
    double Shooterspeed;
    double FlapPos;
    boolean shooterLast = false;
    boolean lowOveride = false;
    boolean highOveride = false;
    boolean highOverrideLast = false;
    boolean lowOverrideLast = false;


    pinpoint.setOffsets(100, -25, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
    pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
    pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
    Flap.setPosition(flapUp);
    telemetry.addData("Status", "Initialized");
    telemetry.update();

    waitForStart();

    while (opModeIsActive())
    {
      telemetry.addData("Status", "Running");
      result = limelight.getLatestResult();
      pinpoint.update();
      telemetry.addData("Heading Scalar", pinpoint.getYawScalar());
      Heading = Math.toRadians(pinpoint.getPosition().getHeading(AngleUnit.DEGREES) + Constants.HeadingOffset);
      telemetry.addData("Heading", Math.toDegrees(Heading));

      double rawForward = gamepad1.left_stick_y; // FTC joystick forward is negative
      double rawStrafe = -gamepad1.left_stick_x;
      Turn = -gamepad1.right_stick_x;

      double sinHeading = Math.sin(-Heading); // Pinpoint heading is CW+, invert for standard CCW math
      double cosHeading = Math.cos(-Heading);

      // Rotate the driver input vector so it is field-centric
      Strafe = rawStrafe * cosHeading - rawForward * sinHeading;
      Forward = rawStrafe * sinHeading + rawForward * cosHeading;
      Shooterspeed = Shooter.getVelocity();

     // FlapPos = gamepad2.left_stick_y;

      if (gamepad1.right_bumper) {
        Forward /= Constants.brake;
        Strafe /= Constants.brake;
        Turn /= Constants.brake;
      }

      if (gamepad2.right_bumper)
      {
        Flap.setPosition(flapDeploy);
      } else
      {
        Flap.setPosition(flapUp);
      }

      if (gamepad1.left_bumper) {
        imu.initialize(new IMU.Parameters((ImuOrientationOnRobot) new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.DOWN, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT)));
        imu.resetYaw();
        pinpoint.resetPosAndIMU(); //resets the position to 0 and recalibrates the IMU
        pinpoint.setHeading(0, AngleUnit.DEGREES);
        pinpoint.update();
      }

      telemetry.addData("PinPoint Status", pinpoint.getDeviceStatus());
      telemetry.addData("forward", Forward);
      telemetry.addData("strafe", Strafe);
      telemetry.addData("Turn", Turn);

      double denominator = Math.max(Math.abs(Forward) + Math.abs(Strafe) + Math.abs(Turn), 1);
      double frontLeftPower = (Forward + Strafe + Turn) / denominator;
      double backLeftPower = (Forward - Strafe + Turn) / denominator;
      double frontRightPower = (Forward - Strafe - Turn) / denominator;
      double backRightPower = (Forward + Strafe - Turn) / denominator;


      fLDrive.setPower(frontLeftPower);
      telemetry.addData("FLDrive", frontLeftPower);
      bLDrive.setPower(backLeftPower);
      telemetry.addData("BLDrive", backLeftPower);
      fRDrive.setPower(frontRightPower);
      telemetry.addData("FRDrive", frontRightPower);
      bRDrive.setPower(backRightPower);
      telemetry.addData("BRDrive", backRightPower);

      telemetry.addData("FRDrive_Actual", fRDrive.getPower());
      telemetry.addData("FLDrive_Actual", fLDrive.getPower());
      telemetry.addData("BRDrive_Actual", bRDrive.getPower());
      telemetry.addData("BLDrive_Actual", bLDrive.getPower());

     // Flap.setPosition(FlapPos);

      Distance detectedDistance = getDetectedColor(telemetry);

      if (gamepad1.b) {
        pinpoint.recalibrateIMU(); //recalibrates the IMU without resetting position
      }

      if (gamepad2.left_trigger > 0)
      {
        if (!shooterLast) {
          shootSequence = !shootSequence;
          shooterStage = 1;
          shooterLast = true;
        }
      } else {
        shooterLast = false;
      }

      if (gamepad2.dpad_down) {
        spindexerPower = spindexerBWD;
      } else {
        spindexerPower = spindexerFWD;
      }

      if (gamepad2.dpad_up) {
        if (!spinToggleLast) {
          spindexerToggle = !spindexerToggle;
          spinToggleLast = true;
        }
      } else {
        spinToggleLast = false;
      }


      if (gamepad2.right_trigger > 0) {
        if (!inToggleLast) {
          intakeToggle = !intakeToggle;
          inToggleLast = true;
        }
      } else {
        inToggleLast = false;
      }

      if (spindexerToggle) {
        SpindxerServo.setPower(spindexerPower);
      } else {
        SpindxerServo.setPower(0);
      }

      if (intakeToggle) {
        Intake.setPower(0.75);
      } else {
        Intake.setPower(0);
      }

      if (result != null && result.isValid()) {
        Pose3D botpose = result.getBotpose();
        if (botpose != null) {
          double x = botpose.getPosition().x;
          double y = botpose.getPosition().y;
          telemetry.addData("MT1 Location", "(" + x + ", " + y + ")");
        }
      }

      // First, tell Limelight which way your robot is facing
      double robotYaw = pinpoint.getHeading(AngleUnit.DEGREES);
      limelight.updateRobotOrientation(robotYaw);
      if (result != null && result.isValid()) {
        Pose3D botPose_mt2 = result.getBotpose_MT2();

        List<FiducialResult> fiducials = result.getFiducialResults();
        for (FiducialResult fiducial : fiducials) {
          int id = fiducial.getFiducialId(); // The ID number of the fiducial
            double x = fiducial.getRobotPoseTargetSpace().getPosition().x; // Horizontal offset (meters)
            double y = fiducial.getRobotPoseTargetSpace().getPosition().y; // Vertical offset (meters)
            double z = fiducial.getRobotPoseTargetSpace().getPosition().z; // Forward distance (meters)
            double StrafeDistance_3D = fiducial.getRobotPoseTargetSpace().getPosition().y;
            double distance = Math.sqrt((x * x) + (z * z)); // Use horizontal plane distance to the tag
            telemetry.addData("Fiducial " + id, "is " + distance + " meters away");

            ShooterTarget = Constants.ShooterCal.interpolate(distance);

        }
      }
      else
      {
        ShooterTarget = Constants.ShooterCal.interpolate(0.2);
      }

      boolean manualShooterRequest = gamepad2.left_bumper;

      if (gamepad2.y)
      {
        if (!highOverrideLast) {
          highOveride = !highOveride;
        }
        highOverrideLast = true;
      } else {
        highOverrideLast = false;
      }

      if (gamepad2.a)
      {
        if (!lowOverrideLast) {
          lowOveride = !lowOveride;
        }
        lowOverrideLast = true;
      } else {
        lowOverrideLast = false;
      }

      if (highOveride) {
       ShooterTarget = 1580;
      }

      if (lowOveride) {
        ShooterTarget = 1420;
      }

      if (shootSequence)
      {
        if (shooterStage < 1 || shooterStage > 4)
        {
          shooterStage = 1;
        }

        if (shooterStage == 1)
        {
          Shooter.setVelocity(ShooterTarget);
          spindexerToggle = true;
          if (detectedDistance == Distance.LOADED)
          {
            lastRuntime = getRuntime();
            shooterStage = 2;
          }
        }
        else if (shooterStage == 2)
        {
          Flap.setPosition(flapDeploy);
          if (getRuntime() - lastRuntime >= 1)
          {
            shooterStage = 3;
          }
        }
        else if (shooterStage == 3)
        {
          spindexerToggle = false;
          if (Shooter.getVelocity() >= ShooterTarget)
          {
            lastRuntime = getRuntime();
            shooterStage = 4;
          }
        }
        else if (shooterStage == 4)
        {
          Flap.setPosition(flapUp);
          // give the flywheel time to coast down before re-arming
          if (Shooter.getVelocity() <= 50 || getRuntime() - lastRuntime >= 1.0)
          {
            Shooter.setVelocity(0);
            shootSequence = false;
            shooterStage = 1;
          }
        }
      } else if (manualShooterRequest)
      {
        Shooter.setVelocity(ShooterTarget);
      } else
      {
        Shooter.setVelocity(0);
        shooterStage = 1;
      }
      telemetry.addData("Shooter Target", ShooterTarget);
      telemetry.addData("Shooter Vel", Shooter.getVelocity());
      telemetry.addData("Shooter Stage", shooterStage);
      telemetry.addData("highOveride", highOveride);
      telemetry.addData("lowOveride", lowOveride);
      telemetry.update();
    }
  }
}






//
