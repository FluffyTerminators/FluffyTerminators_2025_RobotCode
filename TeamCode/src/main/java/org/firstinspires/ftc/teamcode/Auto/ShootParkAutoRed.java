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
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Util.Constants;
import org.firstinspires.ftc.teamcode.Util.Constants.PEDROConstants;
import org.firstinspires.ftc.teamcode.Util.GoBildaPinpointDriver;

import java.util.List;

@Autonomous(name = "Shoot & Park Red")
@Configurable // Panels
public class ShootParkAutoRed extends OpMode {

  private TelemetryManager panelsTelemetry; // Panels Telemetry instance
  public Follower follower; // Pedro Pathing follower instance
  private int pathState; // Current autonomous path state (state machine)
  private Paths paths; // Paths defined in the Paths class
  private Limelight3A limelight;
  private GoBildaPinpointDriver pinpoint;
  private DcMotorEx Shooter;
  private ColorRangeSensor SpindexerSensor1;
  private ColorRangeSensor SpindexerSensor2;
  private Servo Flap;
  private CRServo SpindxerServo;
  private boolean spindexToggle;
  private double ShooterTarget;
  private double lastRunTime = 0;
  private int shooterState = 3;
  private int cyclesAtSpeed = 0;
  private int shotsToTake = 0;
  private int cycleThreshold = 100;

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
    Shooter.setVelocity(ShooterTarget);
    telemetry.addData("Shooter Motor", Shooter);
    telemetry.addData("Spindexer?", spindexToggle);
    telemetry.addData("Shooter Target", ShooterTarget);
    telemetry.addData("Runtime", lastRunTime);
    telemetry.addData("Shooter State", shooterState);
    telemetry.addData("First Colour", SpindexerSensor1);
    telemetry.addData("Second Colour", SpindexerSensor2);
    telemetry.addData("Servo", Flap);

    switch (shooterState) {
      case -1:
        break;

      case 0: // Spin up
        spindexToggle = true;
        shooterState = 1;
        break;

      case 1: // Wait for note detection
        Distance detectedDistance = getDetectedColor();
        if (detectedDistance == Distance.LOADED) {
          lastRunTime = getRuntime();
          shooterState = 2;
        }
        break;

      case 2: // Drop flap to feed note
        spindexToggle = false;
        Flap.setPosition(Constants.flapDeploy);
        if (getRuntime() - lastRunTime > 0.25) {
          lastRunTime = getRuntime();
          shooterState = 3;
          cyclesAtSpeed = 0;
        }
        break;

      case 3: // Stop spindexer and wait for flywheel to slow down
        spindexToggle = true;
        if (getRuntime() - lastRunTime > 1) {
          spindexToggle = false;
          if ((Shooter.getVelocity() > ShooterTarget - 40) && (Shooter.getVelocity() < ShooterTarget +40)) {
            cyclesAtSpeed ++;
          } else {
            cyclesAtSpeed = 0;
          }
          if (cyclesAtSpeed > cycleThreshold) {
            shooterState = 4;
            lastRunTime = getRuntime();
          }
        }
        break;

      case 4: // Retract flap once flywheel recovers
        Flap.setPosition(Constants.flapUp);
        if (getRuntime() - lastRunTime > 0.5) {
          shooterState = 2;
        }
        if (Shooter.getVelocity() < ShooterTarget - 400) {
          Shooter.setVelocity(0);
          shooterState = -1;
        }
        break;

      default:
        break;
    }
  }

  @Override
  public void init() {
    panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    limelight = hardwareMap.get(Limelight3A.class, "Limelight");
    pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
    Shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
    SpindexerSensor1 = hardwareMap.get(ColorRangeSensor.class, "spindexer_colour_1");
    SpindexerSensor2 = hardwareMap.get(ColorRangeSensor.class, "spindexer_colour_2");
    Flap = hardwareMap.get(Servo.class, "Spindexer_Flap_Servo");
    SpindxerServo = hardwareMap.get(CRServo.class, "Spindexer_Servo");
    Flap.setPosition(Constants.flapDeploy);
    lastRunTime = getRuntime();

    follower = PEDROConstants.createFollower(hardwareMap);
    follower.setStartingPose(new Pose(71, 8, Math.toRadians(90)));

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
          double StrafeDistance_3D = fiducial.getRobotPoseTargetSpace().getPosition().y;
          double distance = Math.sqrt((x * x) + (y * y));
          telemetry.addData("Fiducial " + id, "is " + distance + " meters away");

          ShooterTarget = Constants.ShooterCal.interpolate(distance);
        }
      }
    }
    else
    {
      ShooterTarget = Constants.ShooterCal.interpolate(0.2);
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
                      new BezierLine(new Pose(71.000, 8.000), new Pose(63, 24.338))
              )
              .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(65))
              .build();

      ParkMiddle = follower
              .pathBuilder()
              .addPath(
                      new BezierLine(new Pose(64, 24.338), new Pose(63, 59.107))
              )
              .setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(90))
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
        shotsToTake = 1;
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
