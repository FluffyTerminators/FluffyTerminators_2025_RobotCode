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

@Autonomous(name = "DoubleCycleAuto")
@Configurable // Panels
public class DoubleCycleAuto extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.PEDROConstants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(56, 8, Math.toRadians(90)));

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

        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.000, 8.000), new Pose(41.000, 35.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(41.000, 35.000), new Pose(10.000, 35.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(10.000, 35.000), new Pose(56.000, 8.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(110))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.000, 8.000), new Pose(41.000, 60.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(180))
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(41.000, 60.000), new Pose(10.000, 60.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(110))
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(10.000, 60.000), new Pose(56.000, 8.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(110))
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.000, 8.000), new Pose(56.000, 8.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(90))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine

        switch (pathState) {
            case 0:
                follower.followPath(paths.Path1);
                pathState = 1;
                break;
            case 1:
                if (!follower.isBusy()){
                    follower.followPath(paths.Path2);
                    pathState = 2;
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path3);
                    pathState = 3;
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path4);
                    pathState = 4;
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path5);
                    pathState = 5;
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path6);
                    pathState = 6;
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path7);
                    pathState = -1;
                }
                break;
        }
        return pathState;
    }
}
