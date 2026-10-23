package org.firstinspires.ftc.teamcode.Auto;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Util.AutoFunctions;
import org.firstinspires.ftc.teamcode.Util.Constants;
import org.firstinspires.ftc.teamcode.Util.Constants.PEDROConstants;
import org.firstinspires.ftc.teamcode.Util.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Util.ShooterPidTuning;
import org.firstinspires.ftc.teamcode.Util.VisionFunctions;

@Autonomous(name = "Red-Goal-Collect")
@Configurable // Panels
public class ApocGoalCollectRed extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState=0; // Current autonomous path state (state machine)
    private ApocGoalCollectRed.Paths paths; // Paths defined in the Paths class
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

    private double targetOffset;

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

        follower = PEDROConstants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(121, 121, Math.toRadians(216)));

        paths = new ApocGoalCollectRed.Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        fLDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        fRDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        bLDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        fRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        targetOffset = 0;

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

        if(VisionFunctions.goodTagNow()){
            Turn = targetOffset * Constants.autoAim_Gain;
            if (Turn < -1) {Turn = -1;}
            if (Turn > 1) {Turn = 1;}
            if (Math.abs(Turn) < Constants.minAutoTurnSpeed) {Turn = 0;}
        } else{
            Turn = 0;
        }

        if (VisionFunctions.goodTag())
        {
            ShooterTarget = VisionFunctions.getDistance();
        }
        else
        {
            ShooterTarget = 0;
        }
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine



        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.debug("offset", targetOffset);
        panelsTelemetry.debug("GoodTagNow", VisionFunctions.goodTagNow());

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
                                    new Pose(121.000, 121.000),
                                    new Pose(94.000, 115.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(216), Math.toRadians(210))
                    .build();

            Preload = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(94.000, 115.000),
                                    new Pose(90.000, 95.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(210), Math.toRadians(0))
                    .addPath(
                            new BezierLine(
                                    new Pose(90.000, 95.000),
                                    new Pose(87.000, 82.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Collect = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(87.000, 72.000),
                                    new Pose(112.000, 72.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .addPath(
                            new BezierLine(
                                    new Pose(112.000, 72.000),
                                    new Pose(90.000, 110.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(210))
                    .build();

            Finish = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(90.000, 110.000),
                                    new Pose(115.000, 90.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(210), Math.toRadians(0))
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
                follower.followPath(paths.Start, false);
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
                        !follower.isBusy());

                if (VisionFunctions.goodTagNow()) {
                    targetOffset = -(VisionFunctions.getAngle(true));
                }

                if (!follower.isBusy()) {

                    //follower.pausePathFollowing();
                    fLDrive.setPower(-Turn);
                    bLDrive.setPower(-Turn);
                    fRDrive.setPower(Turn);
                    bRDrive.setPower(Turn);
                    telemetry.addData("shotCount", AutoFunctions.shotCount);
                    if (AutoFunctions.shotCount >= 4) {
                        Passthrough.setPower(0);
                        pathState = 2;
                        //follower.resumePathFollowing();
                    }
                }

                break;
            case 2:
                if(!follower.isBusy())
                {
                    follower.followPath(paths.Preload, true);
                    AutoFunctions.resetShotCount();
                    pathState = 3;
                }
                break;
            case 3:
                if(!follower.isBusy())
                {
                    follower.followPath(paths.Collect, false);
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
                        !follower.isBusy());

                if (VisionFunctions.goodTagNow()) {
                    targetOffset = -(VisionFunctions.getAngle(true));
                }


                if (follower.isBusy()) {
                    double intakeSpeed = IntakeEx.getVelocity() / 2380;
                        Intake.setPower(Constants.Intake_In_Speed);
                } else {

                    fLDrive.setPower(-Turn);
                    bLDrive.setPower(-Turn);
                    fRDrive.setPower(Turn);
                    bRDrive.setPower(Turn);
                    telemetry.addData("shotCount", AutoFunctions.shotCount);
                    if (AutoFunctions.shotCount >= 4) {
                        pathState = 5;
                    }

                }
                break;
            case 5:
                ShooterFront.setVelocity(0);
                ShooterBack.setVelocity(0);
                IntakeEx.setPower(0);
                Passthrough.setPower(0);
                follower.followPath(paths.Finish, true);
                pathState = -1;
                break;
        }
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
        return pathState;
    }
}