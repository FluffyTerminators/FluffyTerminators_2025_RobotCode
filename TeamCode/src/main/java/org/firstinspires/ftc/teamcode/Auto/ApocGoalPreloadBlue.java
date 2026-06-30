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
import org.firstinspires.ftc.teamcode.Util.VisionFunctions;

import java.util.List;

@Autonomous(name = "Blue-Goal-Preload")
@Configurable // Panels
public class ApocGoalPreloadBlue extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState=0; // Current autonomous path state (state machine)
    private ApocGoalPreloadBlue.Paths paths; // Paths defined in the Paths class
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

        VisionFunctions.init(limelight,this::getRuntime);

        follower = Constants.PEDROConstants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(20.5, 121, Math.toRadians(324)));

        paths = new Paths(follower); // Build paths

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
        //LLResult result = limelight.getLatestResult();

        /*if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();
            if (botpose != null) {
                double x = botpose.getPosition().x;
                double y = botpose.getPosition().y;
                telemetry.addData("MT1 Location", "(" + x + ", " + y + ")");
            }
        }*/

        // First, tell Limelight which way your robot is facing
        double robotYaw = pinpoint.getHeading(AngleUnit.DEGREES);
        //limelight.updateRobotOrientation(robotYaw);
        VisionFunctions.update(robotYaw);
        Turn = 0;
        /*if (result != null && result.isValid()) {
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
        }*/
        if (VisionFunctions.goodTag())
        {
            ShooterTarget = VisionFunctions.getDistance();
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
        public PathChain StartChain, FinishChain;

        public Paths(Follower follower) {
            StartChain = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(20.500, 121.000),
                                    new Pose(61.500, 115.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(324), Math.toRadians(335))
                    .build();
            FinishChain = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(61.500, 115.000),
                                    new Pose(14.500, 100.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(335), Math.toRadians(180))
                    .build();
        }
    }

    public int autonomousPathUpdate() {

        telemetry.addData("ShotCount",AutoFunctions.shotCount);
        telemetry.addData("ShooterTimer",AutoFunctions.shooterTimer);
        AutoFunctions.isAuto = true;

        // Add your state machine Here
        switch (pathState) {
            case 0:
                follower.followPath(paths.StartChain);
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
                    if (VisionFunctions.goodTagNow())
                    {
                        double headingOffset = VisionFunctions.getAngle(true);
                        if (headingOffset != 0)
                        {
                            follower.turnDegrees(Math.abs(headingOffset),(headingOffset > 0));
                        }
                    }
                    /*fLDrive.setPower(-Turn);
                    bLDrive.setPower(-Turn);
                    fRDrive.setPower(Turn);
                    bRDrive.setPower(Turn);*/
                    if (AutoFunctions.shotCount >= 4) {
                        pathState = 2;
                    }
                }
                break;
            case 2:
                ShooterFront.setVelocity(0);
                ShooterBack.setVelocity(0);
                IntakeEx.setPower(0);
                Passthrough.setPower(0);
                follower.followPath(paths.FinishChain);
                pathState = -1;
                break;
        }
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
        return pathState;
    }
}