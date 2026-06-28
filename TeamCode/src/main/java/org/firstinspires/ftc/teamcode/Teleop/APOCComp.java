package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.hardware.LED;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Util.AutoFunctions;
import org.firstinspires.ftc.teamcode.Util.Constants;
import org.firstinspires.ftc.teamcode.Util.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Util.ShooterPidTuning;
import org.firstinspires.ftc.teamcode.Util.Constants.Toggle;
import java.util.List;
//Download Missing Files

@TeleOp(name = "APOC Comp")
public class APOCComp extends LinearOpMode
{
    // Drive Motors
    public DcMotor bLDrive;
    public DcMotor bRDrive;
    public DcMotor fLDrive;
    public DcMotor fRDrive;

    // Mechanism Motors
    public DcMotor Intake;
    public DcMotorEx IntakeEx;
    public DcMotorEx ShooterFront;
    public DcMotorEx ShooterBack;

    //servo
    public CRServo PassThrough;

    // Internal Motion Units
    public IMU imu;
    public GoBildaPinpointDriver pinpoint;
    public Pose2D RobotPosition = new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.DEGREES, 0);

    public Limelight3A limelight;
    private boolean FrontSuccess;
    private boolean BackSuccess;
    public LED tegLED;

    private Toggle ShooterToggle = new Toggle();
    private Toggle IntakeToggle = new Toggle();
    private Toggle HighToggle = new Toggle();
    private Toggle LowToggle = new Toggle();


    public void runOpMode() throws InterruptedException
    {
        bLDrive = hardwareMap.get(DcMotor.class, "BLDrive");
        bRDrive = hardwareMap.get(DcMotor.class, "BRDrive");
        fLDrive = hardwareMap.get(DcMotor.class, "FLDrive");
        fRDrive = hardwareMap.get(DcMotor.class, "FRDrive");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        IntakeEx = (DcMotorEx) Intake;
        PassThrough = hardwareMap.get(CRServo.class, "passthrough_servo");
        ShooterFront = hardwareMap.get(DcMotorEx.class, "ShooterFront");
        ShooterBack = hardwareMap.get(DcMotorEx.class, "ShooterBack");

        imu = hardwareMap.get(IMU.class, "imu");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        FrontSuccess = ShooterPidTuning.applyTo(ShooterFront, 2);
        BackSuccess = ShooterPidTuning.applyTo(ShooterBack, 3);

        telemetry.addData("FrontSuccess", FrontSuccess);
        telemetry.addData("BackSuccess", BackSuccess);

        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        limelight.pipelineSwitch(Constants.LLPipeline); // Switch to pipeline number 0

        LLResult result = limelight.getLatestResult();


        telemetry.addData("Current Pipeline = ", result.getPipelineIndex());

        fLDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        fRDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        bLDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        fRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ShooterBack.setDirection(DcMotorSimple.Direction.REVERSE);

        ShooterFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ShooterBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu.initialize(new IMU.Parameters((ImuOrientationOnRobot) new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.DOWN, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT)));






        double Forward = 0;
        double Strafe = 0;
        double Turn = 0;
        double Heading = 0;
        boolean runIntake = false;
        double intakeSpeed = 0;
        boolean shootRequest = false;
        boolean revRequest = false;
        int shooterStage = 0;
        double ShooterTarget = 0;
        boolean shooterFiring = false;
        boolean lowOveride = false;
        boolean highOveride = false;
        boolean highOverrideLast = false;
        boolean lowOverrideLast = false;
        int pipeline = Constants.LLPipeline;
        boolean pipelineUpLast = false;
        boolean pipelineDownLast = false;
        boolean fieldCentricMode = true;
        double fieldCentricTimer = 0;
        double shooterTimer = 0;
        int cyclesAtSpeed = 0;
        boolean resetLast = false;

        pinpoint.setOffsets(100, -25, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.setHeading(0, AngleUnit.DEGREES);
        pinpoint.update();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            AutoFunctions.isAuto = false;

            //Apply PID Tuning if application of settings failed before OpMode
            if (!FrontSuccess) { FrontSuccess = ShooterPidTuning.applyTo(ShooterFront); }
            if (!BackSuccess) { BackSuccess = ShooterPidTuning.applyTo(ShooterBack); }

            //Update and/or reset Gyro Heading
            if (gamepad1.left_bumper) {
                if (!resetLast) {
                    //imu.resetYaw();
                    pinpoint.setHeading(0, AngleUnit.DEGREES);
                    resetLast = true;
                }
            } else { resetLast = false; }
            pinpoint.update();

            //Calculate Heading
            double robotYaw = pinpoint.getHeading(AngleUnit.DEGREES);
            Heading = Math.toRadians(pinpoint.getPosition().getHeading(AngleUnit.DEGREES) + Constants.HeadingOffset);

            //Display Heading info
            telemetry.addData("PinPoint Status", pinpoint.getDeviceStatus());
            telemetry.addData("Heading Scalar", pinpoint.getYawScalar());
            telemetry.addData("Heading", Math.toDegrees(Heading));


            //Toggle Drive mode (button must be held)
            if (gamepad1.b) {
                if (fieldCentricTimer == 0) {
                    fieldCentricTimer = getRuntime();
                } else {
                    if ((fieldCentricTimer > 0) && (getRuntime() - fieldCentricTimer > 0.5)) {
                        fieldCentricMode = !fieldCentricMode;
                        fieldCentricTimer = -1;
                    }
                }
            } else {
                fieldCentricTimer = 0;
            }

            //Read joystick inputs
            double rawForward = gamepad1.left_stick_y; // FTC joystick forward is negative
            double rawStrafe = -gamepad1.left_stick_x;
            Turn = -gamepad1.right_stick_x;

            //Calculate base movement values
            if (fieldCentricMode) {
                telemetry.addData("Drive Mode:", "Field Centric");

                double sinHeading = Math.sin(-Heading); // Pinpoint heading is CW+, invert for standard CCW math
                double cosHeading = Math.cos(-Heading);

                // Rotate the driver input vector so it is field-centric
                Forward = rawStrafe * sinHeading + rawForward * cosHeading;
                Strafe = rawStrafe * cosHeading - rawForward * sinHeading;
            } else {
                telemetry.addData("Drive Mode", "Robot Centric");
                Forward = rawForward;
                Strafe = rawStrafe;
            }

            //Apply Brakes
            if (gamepad1.right_bumper) {
                Forward *= Constants.brake;
                Strafe *= Constants.brake;
                Turn *= Constants.brake;
            }

            //Auto Aim
            if (gamepad1.right_stick_button) {
                if ((result != null) && (result.isValid())) {
                    List<FiducialResult> fiducials = result.getFiducialResults();
                    for (FiducialResult fiducial : fiducials) {
                        int id = fiducial.getFiducialId(); // The ID number of the fiducial
                        if ((id == 20) || (id == 24)) {
                            double targetOffset = -fiducial.getTargetXDegrees();
                            Turn = targetOffset / Constants.autoAim_Gain;
                            if (Turn < -1) {Turn = -1;}
                            if (Turn > 1) {Turn = 1;}
                            if (Math.abs(Turn) < 0.05) {Turn = 0;}
                        }
                    }
                }
            }

            //Display Movement Values
            telemetry.addData("Forward", Forward);
            telemetry.addData("Strafe", Strafe);
            telemetry.addData("Turn", Turn);

            //Convert Movement values into motor power
            double denominator = Math.max(Math.abs(Forward) + Math.abs(Strafe) + Math.abs(Turn), 1);
            double frontLeftPower = (Forward + Strafe + Turn) / denominator;
            double backLeftPower = (Forward - Strafe + Turn) / denominator;
            double frontRightPower = (Forward - Strafe - Turn) / denominator;
            double backRightPower = (Forward + Strafe - Turn) / denominator;

            //Set drive motor power
            fLDrive.setPower(frontLeftPower);
            bLDrive.setPower(backLeftPower);
            fRDrive.setPower(frontRightPower);
            bRDrive.setPower(backRightPower);

            //Limelight calculations

            //Change pipeline
            if (gamepad1.dpad_up)
            { if (!pipelineUpLast) {
                pipeline = pipeline + 1;
                limelight.pipelineSwitch(pipeline);
                pipelineUpLast = true;
            }
            } else
            { pipelineUpLast = false; }

            if (gamepad1.dpad_down)
            { if (!pipelineDownLast) {
                pipeline = pipeline - 1;
                limelight.pipelineSwitch(pipeline);
                pipelineDownLast = true;
            }
            } else
            { pipelineDownLast = false; }

            //Tell Limelight which way the robot is facing
            limelight.updateRobotOrientation(robotYaw);
            result = limelight.getLatestResult();

            if (result != null) {
                telemetry.addData("Current Pipeline = ", result.getPipelineIndex());
                if (result.isValid()){

                    //Update Position
                    Pose3D botPose = result.getBotpose();
                    Pose3D botPose_mt2 = result.getBotpose_MT2();
                    if (botPose != null) {
                        double x = botPose.getPosition().x;
                        double z = botPose.getPosition().z;
                        telemetry.addData("MT1 Location", "(" + x + ", " + z + ")");
                    } else{
                        telemetry.addData("MT1 Location", "Not Found");
                    }

                    //Get Distance from Target
                    List<FiducialResult> fiducials = result.getFiducialResults();
                    for (FiducialResult fiducial : fiducials) {
                        int id = fiducial.getFiducialId(); // The ID number of the fiducial
                        if ((id == 20) || (id == 24)) {
                            double x = fiducial.getRobotPoseTargetSpace().getPosition().x; // Horizontal offset (meters)
                            double y = fiducial.getRobotPoseTargetSpace().getPosition().y; // Vertical offset (meters)
                            double z = fiducial.getRobotPoseTargetSpace().getPosition().z; // Forward distance (meters)
                            double StrafeDistance_3D = fiducial.getRobotPoseTargetSpace().getPosition().y;
                            double distance = Math.sqrt((x * x) + (z * z)); // Use horizontal plane distance to the tag
                            telemetry.addData("Fiducial " + id, "is " + distance + " meters away");

                            ShooterTarget = distance;
                        }
                    }
                } else {
                    ShooterTarget = 3.75;
                    telemetry.addData("MT1 Location", "Not Found");
                    telemetry.addData("Fiducial", "Not Found");
                }
            }


            //Shooter Controls
            shootRequest = gamepad2.left_bumper;
            revRequest = ShooterToggle.toggleInput((gamepad2.left_trigger > 0),revRequest);

            //Check Shooter Range Overrides
            if (gamepad2.y)
            { if (!highOverrideLast)
                {
                    lowOveride = false;
                    highOveride = !highOveride;
                    highOverrideLast = true;
                }
            } else { highOverrideLast = false; }

            if (gamepad2.a)
            { if (!lowOverrideLast)
                {
                    lowOveride = !lowOveride;
                    highOveride = false;
                    lowOverrideLast = true;
                }
            } else {
                lowOverrideLast = false; }

            //Apply Shooter Range Overrides
            if (highOveride) { ShooterTarget = Constants.High_Override_Range; }
            if (lowOveride) { ShooterTarget = Constants.Low_Override_Range; }




            AutoFunctions.runShooter(ShooterFront,
                    ShooterBack,
                    IntakeEx,
                    ShooterTarget,
                    getRuntime(),
                    PassThrough,
                    shootRequest,
                    revRequest);

            telemetry.addData("Shooter State", AutoFunctions.shooterState);

            switch (AutoFunctions.shooterState){
                case 0: {
                    telemetry.addData("Shooter Status", "*  Too Close!  *");
                    shooterFiring = false;
                    break;
                }
                case 1: {
                    telemetry.addData("Shooter Status", "*** Firing! ***");
                    shooterFiring = true;
                    break;
                }
                case 2: {
                    telemetry.addData("Shooter Status", "***  Ready  ***");
                    shooterFiring = false;
                    break;
                }
                case 3: {
                    telemetry.addData("Shooter Status", "Waiting");
                    shooterFiring = false;
                    break;
                }
                case 4: {
                    telemetry.addData("Shooter Status", "Spinning up...");
                    shooterFiring = false;
                    break;
                }
                case 5: {
                    telemetry.addData("Shooter Status", "***   Idle   ***");
                    shooterFiring = false;
                    break;
                }
            }

            //Intake Control
            runIntake = IntakeToggle.toggleInput((gamepad2.right_trigger > 0),runIntake);

            intakeSpeed = IntakeEx.getVelocity() / 2380;

            //Set Intake power
            if (runIntake) {
                if (gamepad2.right_bumper)
                    { Intake.setPower(Constants.Intake_Eject_Speed);} //Eject Balls
                    else {
                        if (intakeSpeed > 0.15) {
                            Intake.setPower(0.2);
                        } else{
                            Intake.setPower(Constants.Intake_In_Speed);
                        }
                    } //Intake Balls
            } else if (!shooterFiring) {
                Intake.setPower(0);
            }

            //PassThrough Control
            if (gamepad2.dpad_down) {
                PassThrough.setPower(-1);
            } else if (gamepad2.dpad_up) {
                PassThrough.setPower(1);
            } else {
                if (!shooterFiring) {
                    if (runIntake)
                    {
                        if (intakeSpeed < (Constants.Intake_In_Speed * 0.7)) {
                            PassThrough.setPower(-0.3);
                        } else {
                            PassThrough.setPower(-0.1);
                        }
                    }
                    else
                    {PassThrough.setPower(0);}
                }
            }



            telemetry.addData("Shot Count", AutoFunctions.shotCount);
            telemetry.addData("Shooter Target Distance", ShooterTarget);
            telemetry.addData("Shooter Target Velocity", AutoFunctions.shooterFTarget);
            telemetry.addData("Shooter Front Vel", ShooterFront.getVelocity());
            telemetry.addData("Shooter Back Vel", ShooterBack.getVelocity());
            telemetry.addData("Shooter Stage", shooterStage);
            telemetry.addData("highOveride", highOveride);
            telemetry.addData("lowOveride", lowOveride);
            telemetry.addData("Shooter Front PID", ShooterFront.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
            telemetry.addData("Shooter Back PID", ShooterBack.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
            telemetry.addData("FrontSuccess", FrontSuccess);
            telemetry.addData("BackSuccess", BackSuccess);
            telemetry.update();

            //Shooter process (for all uses of shooter)
            //Set motor speeds to interpolated target speed
            //if inside range, run passthrough and intake

            //if needed, use timer to require motor speeds to be inside target range for a set duration before indexing

            //for now, end on button release (happens automatically when variable is unset)
            //when shootSequence is implemented, use timer code to track when shooter reaches speed
            //end after set duration past this point




        }
    }
}
