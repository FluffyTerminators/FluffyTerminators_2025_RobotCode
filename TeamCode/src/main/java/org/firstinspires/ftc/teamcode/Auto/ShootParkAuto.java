package org.firstinspires.ftc.teamcode.Auto;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Util.Constants;
import org.firstinspires.ftc.teamcode.Util.Constants.PEDROConstants;

@Autonomous(name = "Shoot & Park")
@Configurable // Panels
public class ShootParkAuto extends OpMode {

  private TelemetryManager panelsTelemetry; // Panels Telemetry instance
  public Follower follower; // Pedro Pathing follower instance
  private int pathState; // Current autonomous path state (state machine)
  private Paths paths; // Paths defined in the Paths class

  @Override
  public void init() {
    panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    follower = Constants.PEDROConstants.createFollower(hardwareMap);
    follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

    paths = new Paths(follower); // Build paths

    panelsTelemetry.debug("Status", "Initialized");
    panelsTelemetry.update(telemetry);
  }

  @Override
  public void loop() {
    follower.update(); // Update Pedro Pathing
    pathState = autonomousPathUpdate(); // Update autonomous state machine

    // Log values to Panels and Driver Station
    panelsTelemetry.debug("Path State", pathState);
    panelsTelemetry.debug("X", follower.getPose().getX());
    panelsTelemetry.debug("Y", follower.getPose().getY());
    panelsTelemetry.debug("Heading", follower.getPose().getHeading());
    panelsTelemetry.update(telemetry);
  }

  public static class Paths {

    public PathChain LaunchCorner;
    public PathChain ParkMiddle;

    public Paths(Follower follower) {
      LaunchCorner = follower
              .pathBuilder()
              .addPath(
                      new BezierLine(new Pose(56.000, 8.000), new Pose(63.163, 24.338))
              )
              .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(120))
              .build();

      ParkMiddle = follower
              .pathBuilder()
              .addPath(
                      new BezierLine(new Pose(63.163, 24.338), new Pose(63.163, 59.107))
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
