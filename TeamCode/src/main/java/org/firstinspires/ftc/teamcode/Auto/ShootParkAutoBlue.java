package org.firstinspires.ftc.teamcode.Auto;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Util.Constants;
import org.firstinspires.ftc.teamcode.Util.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Util.ShooterPidTuning;
import java.util.List;

@Autonomous(name = "Shoot & Park Blue")
@Configurable // Panels
public class ShootParkAutoBlue extends OpMode
{
  private TelemetryManager panelsTelemetry; // Panels Telemetry instance
  public Follower follower; // Pedro Pathing follower instance
  private int pathState; // Current autonomous path state (state machine)
  private Paths paths; // Paths defined in the Paths class
  private Limelight3A limelight;
  private GoBildaPinpointDriver pinpoint;
  private DcMotorEx ShooterFront;
  private DcMotorEx ShooterBack;
  private ColorRangeSensor SpindexerSensor1;
  private ColorRangeSensor SpindexerSensor2;
  private CRServo SpindxerServo;
  private boolean spindexToggle;
  private double ShooterTarget;
  private double lastRunTime = 0;
  private int shooterState = 0;
  private int cyclesAtSpeed = 0;
  private int shotsToTake = 0;
  private int cycleThreshold = 5;
  private static final double VELOCITY_TOLERANCE = 50.0; // ticks/s tolerance
  private static final double FEED_TIME = 0.25; // seconds to feed note
  private static final double RECOVER_TIME = 0.2; // seconds to allow flywheel to bounce back

  public enum Distance {
    LOADED,
    EMPTY
  }

  private static final double OBJECT_DETECTION_RANGE_CM = 4.0;

  public Distance getDetectedColor() {
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

  public void runShooter()
  {
    ShooterFront.setVelocity(ShooterTarget);
    ShooterBack.setVelocity(ShooterTarget);
    telemetry.addData("ShooterFront Motor", ShooterFront);
    telemetry.addData("ShooterBack Motor", ShooterBack);
    telemetry.addData("Spindexer?", spindexToggle);
    telemetry.addData("Shooter Target", ShooterTarget);
    telemetry.addData("Runtime", lastRunTime);
    telemetry.addData("Shooter State", shooterState);
    telemetry.addData("First Colour", SpindexerSensor1);
    telemetry.addData("Second Colour", SpindexerSensor2);

    switch (shooterState) {
      case -1:
        spindexToggle = false;
        break;

      case 0: // Spin up while driving to position
        spindexToggle = false;
        if (isAtTargetVelocity()) {
          cyclesAtSpeed++;
        } else {
          cyclesAtSpeed = 0;
        }
        if (cyclesAtSpeed > cycleThreshold) {
          shooterState = 1;
          lastRunTime = getRuntime();
          cyclesAtSpeed = 0;
        }
        break;

      case 1: // Feed one note
        spindexToggle = true;
        if (getRuntime() - lastRunTime > FEED_TIME) {
          spindexToggle = false;
          shooterState = 2;
          lastRunTime = getRuntime();
          cyclesAtSpeed = 0;
        }
        break;

      case 2: // Let flywheel recover then mark shot complete
        if (isAtTargetVelocity()) {
          cyclesAtSpeed++;
        } else {
          cyclesAtSpeed = 0;
        }
        if (cyclesAtSpeed > cycleThreshold || getRuntime() - lastRunTime > RECOVER_TIME) {
          shooterState = -1;
        }
        break;

      default:
        break;
    }
  }

  private boolean isAtTargetVelocity() {
    double front = ShooterFront.getVelocity();
    double back = ShooterBack.getVelocity();
    boolean frontOk = (front > ShooterTarget - VELOCITY_TOLERANCE) && (front < ShooterTarget + VELOCITY_TOLERANCE);
    boolean backOk = (back > ShooterTarget - VELOCITY_TOLERANCE) && (back < ShooterTarget + VELOCITY_TOLERANCE);
    return frontOk && backOk;
  }
  @Override
  public void init() {
    panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    limelight = hardwareMap.get(Limelight3A.class, "Limelight");
    pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
    ShooterFront = hardwareMap.get(DcMotorEx.class, "ShooterFront");
    ShooterBack = hardwareMap.get(DcMotorEx.class, "ShooterBack");
    SpindexerSensor1 = hardwareMap.get(ColorRangeSensor.class, "spindexer_colour_1");
    SpindexerSensor2 = hardwareMap.get(ColorRangeSensor.class, "spindexer_colour_2");
    SpindxerServo = hardwareMap.get(CRServo.class, "Spindexer_Servo");
    lastRunTime = getRuntime();
    limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
    limelight.start(); // This tells Limelight to start looking!
    limelight.pipelineSwitch(7); // Switch to pipeline number 0
    ShooterBack.setDirection(DcMotorSimple.Direction.REVERSE);
    ShooterFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    ShooterBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    ShooterFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    ShooterBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    ShooterPidTuning.applyTo(ShooterFront);
    ShooterPidTuning.applyTo(ShooterBack);

    follower = Constants.PEDROConstants.createFollower(hardwareMap);
    follower.setStartingPose(new Pose(56, 8, Math.toRadians(90)));

    paths = new Paths(follower); // Build paths

    panelsTelemetry.debug("Status", "Initialized");
    panelsTelemetry.update(telemetry);
  }

  @Override
  public void loop() {
    LLResult result = limelight.getLatestResult();

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

      List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
      if (fiducials != null && !fiducials.isEmpty()) {
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
          int id = fiducial.getFiducialId(); // The ID number of the fiducial
          double x = fiducial.getRobotPoseTargetSpace().getPosition().x; // Where it is (left-right)
          double y = fiducial.getRobotPoseTargetSpace().getPosition().y; // Where it is (up-down)
          double z = fiducial.getRobotPoseTargetSpace().getPosition().z;
          double StrafeDistance_3D = fiducial.getRobotPoseTargetSpace().getPosition().y;
          double distance = Math.sqrt((x * x) + (z * z));
          telemetry.addData("Fiducial " + id, "is " + distance + " meters away");

          ShooterTarget = Constants.ShooterCal.interpolate(distance);
        }
      }
    }
    else
    {
      ShooterTarget = Constants.ShooterCal.interpolate(2.2);
    }
    follower.update(); // Update Pedro Pathing
    pathState = autonomousPathUpdate(); // Update autonomous state machine

    // Log values to Panels and Driver Station
    panelsTelemetry.debug("Path State", pathState);
    panelsTelemetry.debug("X", follower.getPose().getX());
    panelsTelemetry.debug("Y", follower.getPose().getY());
    panelsTelemetry.debug("Heading", follower.getPose().getHeading());
    panelsTelemetry.update(telemetry);

    if (spindexToggle) {
      SpindxerServo.setPower(1);
    } else {
      SpindxerServo.setPower(0);
    }
  }

  public static class Paths {

    public PathChain LaunchCorner;
    public PathChain ParkMiddle;

    public Paths(Follower follower) {
      LaunchCorner = follower
              .pathBuilder()
              .addPath(
                      new BezierLine(new Pose(56.000, 8.000), new Pose(63, 24.338))
              )
              .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(115))
              .build();

      ParkMiddle = follower
              .pathBuilder()
              .addPath(
                      new BezierLine(new Pose(63, 24.338), new Pose(63, 59.107))
              )
              .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(90))
              .build();
    }
  }
  public int autonomousPathUpdate() {
    // Add your state machine Here

    switch (pathState)
    {
      case 0:
        follower.followPath(paths.LaunchCorner);
        pathState = 1;
        shotsToTake = 3;
        shooterState = 0; // start spin-up immediately
        break;

      case 1:
        runShooter();
        if (shooterState == -1)
        {
          shotsToTake --;
          if (shotsToTake > 0) {
            shooterState = 0;
          } else {
            pathState = 2;
          }
        }
        break;

      case 2:
        if (!follower.isBusy()){

          follower.followPath(paths.ParkMiddle);
          pathState = 3;
          ShooterFront.setVelocity(0);
          ShooterBack.setVelocity(0);
        }
        break;

      case 3:
        spindexToggle = false;
        pathState = -1;
        break;


      default:
        break;
    }
    // Access paths with paths.pathName
    // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
    return pathState;
  }
}
