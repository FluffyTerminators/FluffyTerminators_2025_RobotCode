package org.firstinspires.ftc.teamcode.Auto;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Util.Constants;
import org.firstinspires.ftc.teamcode.Util.Constants.PEDROConstants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Util.AutoFunctions;
import org.firstinspires.ftc.teamcode.Util.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Util.ShooterPidTuning;

import java.util.List;
@Autonomous(name = "Red-Back-Collect")
@Configurable // Panels
public class ApocCollectRed extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState=0; // Current autonomous path state (state machine)
    private ApocCollectRed.Paths paths; // Paths defined in the Paths class
    private Limelight3A limelight;
    private GoBildaPinpointDriver pinpoint;
    private DcMotorEx ShooterFront;
    private DcMotorEx ShooterBack;
    private DcMotor Intake;
    public DcMotorEx IntakeEx;
    public DcMotor bLDrive;
    public DcMotor bRDrive;
    public DcMotor fLDrive;
    public DcMotor fRDrive;
    private boolean intakeToggle;
    private CRServo Passthrough;
    private boolean passthroughToggle;
    private double ShooterTarget;
    private double shotsToTake;
    private double runtime;

    private static double Turn;

    @Override
    public void init() {
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        IntakeEx = (DcMotorEx) Intake;
        Passthrough = hardwareMap.get(CRServo.class, "passthrough_servo");
        ShooterFront = hardwareMap.get(DcMotorEx.class, "ShooterFront");
        ShooterBack = hardwareMap.get(DcMotorEx.class, "ShooterBack");
        bLDrive = hardwareMap.get(DcMotor.class, "BLDrive");
        bRDrive = hardwareMap.get(DcMotor.class, "BRDrive");
        fLDrive = hardwareMap.get(DcMotor.class, "FLDrive");
        fRDrive = hardwareMap.get(DcMotor.class, "FRDrive");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        runtime = getRuntime();
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        limelight.pipelineSwitch(Constants.LLPipeline); // Switch to pipeline number 0
        ShooterBack.setDirection(DcMotorSimple.Direction.REVERSE);
        ShooterFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ShooterBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ShooterPidTuning.applyTo(ShooterFront, 2);
        ShooterPidTuning.applyTo(ShooterBack, 3);

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.PEDROConstants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(87, 8, Math.toRadians(270)));

        paths = new ApocCollectRed.Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        fLDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        fRDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        bLDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        fRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

        double robotYaw = pinpoint.getHeading(AngleUnit.DEGREES);
        limelight.updateRobotOrientation(robotYaw);
        Turn = 0;
        if (result != null && result.isValid()) {
            Pose3D botPose_mt2 = result.getBotpose_MT2();

            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            //if (fiducials != null && !fiducials.isEmpty()) {
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                int id = fiducial.getFiducialId(); // The ID number of the fiducial
                double x = fiducial.getRobotPoseTargetSpace().getPosition().x; // Where it is (left-right)
                double y = fiducial.getRobotPoseTargetSpace().getPosition().y; // Where it is (up-down)
                double z = fiducial.getRobotPoseTargetSpace().getPosition().z;
                double StrafeDistance_3D = fiducial.getRobotPoseTargetSpace().getPosition().y;
                double distance = Math.sqrt((x * x) + (z * z));
                telemetry.addData("Fiducial " + id, "is " + distance + " meters away");

                if ((id == 20) || (id == 24)) {
                    ShooterTarget = distance;

                    double targetOffset = -fiducial.getTargetXDegrees();
                    Turn = targetOffset / Constants.autoAim_Gain;
                    if (Turn < -1) {
                        Turn = -1;
                    }
                    if (Turn > 1) {
                        Turn = 1;
                    }
                    if (Math.abs(Turn) < 0.05) {
                        Turn = 0;
                    }
                }
            }
            //}
        }
        else
        {
            ShooterTarget = 3.75;
        }
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
        public PathChain Start;
        public PathChain Preload;
        public PathChain Collect;
        public PathChain Finish;

        public Paths(Follower follower) {
            Start = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(87.000, 8.000),
                                    new Pose(73.000, 27.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(220))
                    .build();

            Preload = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(73.000, 27.000),
                                    new Pose(80.000, 30.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(220), Math.toRadians(0))
                    .build();

            Collect = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(80.000, 30.000),
                                    new Pose(110.000, 30.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .addPath(
                            new BezierLine(
                                    new Pose(110.000, 30.000),
                                    new Pose(73.000, 27.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(220))
                    .build();

            Finish = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(73.000, 27.000),
                                    new Pose(115.000, 15.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(220), Math.toRadians(0))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        // Add your state machine Here
        switch (pathState) {
            case 0:
                follower.followPath(paths.Start);
                AutoFunctions.resetShotCount();
                pathState = 1;
                break;
            case 1:
                AutoFunctions.runShooter(ShooterFront,
                        ShooterBack,
                        IntakeEx,
                        ShooterTarget,
                        getRuntime(),
                        Passthrough,
                        !follower.isBusy(),
                        true);

                if (!follower.isBusy()) {
                    follower.pausePathFollowing();
                    fLDrive.setPower(-Turn);
                    bLDrive.setPower(-Turn);
                    fRDrive.setPower(Turn);
                    bRDrive.setPower(Turn);
                    telemetry.addData("shotCount", AutoFunctions.shotCount);
                    if (AutoFunctions.shotCount >= 3) {
                        Passthrough.setPower(0);
                        pathState = 2;
                        follower.resumePathFollowing();
                    }
                }
                break;
            case 2:
                if(!follower.isBusy())
                {
                    follower.followPath(paths.Preload);
                    AutoFunctions.resetShotCount();
                    pathState = 3;
                }
                break;
            case 3:
                if(!follower.isBusy())
                {
                    follower.followPath(paths.Collect);
                    AutoFunctions.resetShotCount();
                    pathState = 4;
                }
                break;
            case 4:
                AutoFunctions.runShooter(ShooterFront,
                        ShooterBack,
                        IntakeEx,
                        ShooterTarget,
                        getRuntime(),
                        Passthrough,
                        !follower.isBusy(),
                        true);

                if (!follower.isBusy()) {

                    fLDrive.setPower(-Turn);
                    bLDrive.setPower(-Turn);
                    fRDrive.setPower(Turn);
                    bRDrive.setPower(Turn);
                    telemetry.addData("shotCount", AutoFunctions.shotCount);
                    if (AutoFunctions.shotCount >= 3) {
                        pathState = 5;
                    }
                }
                break;
            case 5:
                ShooterFront.setVelocity(0);
                ShooterBack.setVelocity(0);
                IntakeEx.setPower(0);
                Passthrough.setPower(0);
                follower.followPath(paths.Finish);
                pathState = -1;
                break;
        }
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
        return pathState;
    }
}