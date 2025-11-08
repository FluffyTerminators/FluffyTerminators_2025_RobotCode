package org.firstinspires.ftc.teamcode.Auto;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Util.AutoFunctions;
import org.firstinspires.ftc.teamcode.Util.Constants;
import org.firstinspires.ftc.teamcode.Util.Constants.PEDROConstants;
import org.firstinspires.ftc.teamcode.Util.GoBildaPinpointDriver;
import java.util.List;
import org.firstinspires.ftc.teamcode.Util.AutoFunctions.*;

@Autonomous(name = "Shoot & Park")
@Configurable // Panels
public class ShootParkAuto extends OpMode {

  private TelemetryManager panelsTelemetry; // Panels Telemetry instance
  public Follower follower; // Pedro Pathing follower instance
  private int pathState; // Current autonomous path state (state machine)
  private Paths paths; // Paths defined in the Paths class
  private Limelight3A limelight;
  private GoBildaPinpointDriver pinpoint;
  private DcMotorEx Shooter;
  private NormalizedColorSensor SpindexerSensor1;
  private NormalizedColorSensor SpindexerSensor2;
  private Servo Flap;
  private CRServo SpindxerServo;
  private boolean toggle;
  private double ShooterTarget;
  private double lastRunTime = 0;
  private double shooterState = 0;

  @Override
  public void init() {
    panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    limelight = hardwareMap.get(Limelight3A.class, "Limelight");
    pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
    Shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
    SpindexerSensor1 = hardwareMap.get(NormalizedColorSensor.class, "spindexer_colour_1");
    SpindexerSensor2 = hardwareMap.get(NormalizedColorSensor.class, "spindexer_colour_2");
    Flap = hardwareMap.get(Servo.class, "Spindexer_Flap_Servo");
    SpindxerServo = hardwareMap.get(CRServo.class, "Spindexer_Servo");

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
          double StrafeDistance_3D = fiducial.getRobotPoseTargetSpace().getPosition().y;
          double distance = Math.sqrt((x * x) + (y * y));
          telemetry.addData("Fiducial " + id, "is " + distance + " meters away");

          ShooterTarget = Constants.ShooterCal.interpolate(distance);
        }
      } else {
        ShooterTarget = Constants.ShooterCal.interpolate(0.2);
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

    if (toggle) {
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
              .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(120))
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
        break;

      case 1:
        AutoFunctions.runShooter(Shooter,
                toggle,
                ShooterTarget,
                0,
                shooterState,
                SpindexerSensor1,
                SpindexerSensor2,
                Flap);
        if (shooterState == -1)
        {
          pathState = 2;
        }
        break;

      case 2:
        if (!follower.isBusy()){

          follower.followPath(paths.ParkMiddle);
          pathState = -1;
        }
        break;
    }
    // Access paths with paths.pathName
    // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
    return pathState;
  }
}
