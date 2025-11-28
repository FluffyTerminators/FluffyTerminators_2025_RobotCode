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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Util.Constants;
import org.firstinspires.ftc.teamcode.Util.Constants.PEDROConstants;
import org.firstinspires.ftc.teamcode.Util.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Util.ShooterPidTuning;

import java.util.List;

@Autonomous(name = "FrontR Shoot & Park")
@Configurable // Panels
public class ShootParkAutoRed extends OpMode {

  private TelemetryManager panelsTelemetry; // Panels Telemetry instance
  public Follower follower; // Pedro Pathing follower instance
  private int pathState; // Current autonomous path state (state machine)
  private Paths paths; // Paths defined in the Paths class
  private Limelight3A limelight;
  private GoBildaPinpointDriver pinpoint;
  private DcMotorEx ShooterFront;
  private DcMotorEx ShooterBack;
  private Servo Flap;
  private CRServo SpindxerServo;
  private boolean spindexToggle;
  private double ShooterTarget;
  private double lastRunTime = 0;
  private int shooterState = 3;
  private int cyclesAtSpeed = 0;
  private int shotsToTake = 0;
  private int cycleThreshold = 100;

  public void runShooter()
  {
    ShooterFront.setVelocity(ShooterTarget);
    ShooterBack.setVelocity(ShooterTarget);
    telemetry.addData("ShooterFront Motor", ShooterFront);
    telemetry.addData("Spindexer?", spindexToggle);
    telemetry.addData("ShooterFront Target", ShooterTarget);
    telemetry.addData("Runtime", lastRunTime);
    telemetry.addData("ShooterFront State", shooterState);
    telemetry.addData("Servo", Flap);

    switch (shooterState) {
      case -1:
        break;

      case 0: // Spin up
        spindexToggle = false;
        ShooterFront.setVelocity(ShooterTarget);
        ShooterBack.setVelocity(ShooterTarget);
        shooterState = 1;
        break;

      case 1: // Wait for note detection
        ShooterFront.setVelocity(ShooterTarget);
        ShooterBack.setVelocity(ShooterTarget);
        if (
                (ShooterFront.getVelocity() > ShooterTarget - 40) && (ShooterFront.getVelocity() < ShooterTarget +40) &&
                        (ShooterBack.getVelocity() > ShooterTarget - 40) && (ShooterBack.getVelocity() < ShooterTarget +40)
        )
        {
          cyclesAtSpeed ++;
        } else {
          cyclesAtSpeed = 0;
        }
        if (cyclesAtSpeed > 6) {
          shooterState = 2;
          lastRunTime = getRuntime();
        }
        break;

      case 2: // Drop flap to feed note
        spindexToggle = true;
        if ((ShooterFront.getVelocity() < ShooterTarget - 100) && (ShooterBack.getVelocity() < ShooterTarget - 100))
        {
          spindexToggle = false;
          shooterState = 3;
        }
        break;

      case 3: // Stop spindexer and wait for flywheel to slow down
        ShooterFront.setVelocity(0);
        ShooterBack.setVelocity(0);
        shooterState = -1;
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
    ShooterFront = hardwareMap.get(DcMotorEx.class, "ShooterFront");
    ShooterBack = hardwareMap.get(DcMotorEx.class, "ShooterBack");
    SpindxerServo = hardwareMap.get(CRServo.class, "Spindexer_Servo");
    lastRunTime = getRuntime();
    limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
    limelight.start(); // This tells Limelight to start looking!
    limelight.pipelineSwitch(Constants.LLPipeline); // Switch to pipeline number 0
    ShooterBack.setDirection(DcMotorSimple.Direction.REVERSE);
    ShooterFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    ShooterBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    ShooterPidTuning.applyTo(ShooterFront);
    ShooterPidTuning.applyTo(ShooterBack);

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
                      new BezierLine(new Pose(71.000, 8.000), new Pose(63, 20))
              )
              .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(65))
              .build();

      ParkMiddle = follower
              .pathBuilder()
              .addPath(
                      new BezierLine(new Pose(64, 20), new Pose(63, 59.107))
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
        shotsToTake = 3;
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
