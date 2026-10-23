package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Util.AutoFunctions;
import org.firstinspires.ftc.teamcode.Util.Constants;
import org.firstinspires.ftc.teamcode.Util.GoBildaPinpointDriver;

@TeleOp(name = "BioTest")
public class BioBuzzTestDrive extends LinearOpMode {
    public DcMotor bLDrive;
    public DcMotor bRDrive;
    public DcMotor fLDrive;
    public DcMotor fRDrive;
    public DcMotorEx intake;
    public DcMotorEx ShFront;
    public GoBildaPinpointDriver pinpoint;

    public void runOpMode() throws InterruptedException {
        bLDrive = hardwareMap.get(DcMotor.class, "BLDrive");
        bRDrive = hardwareMap.get(DcMotor.class, "BRDrive");
        fLDrive = hardwareMap.get(DcMotor.class, "FLDrive");
        fRDrive = hardwareMap.get(DcMotor.class, "FRDrive");
        intake = hardwareMap.get(DcMotorEx.class, "INTAKE");
        ShFront = hardwareMap.get(DcMotorEx.class, "ShooterFront");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "PinPoint");

        fLDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        fRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double Forward = 0;
        double Strafe = 0;
        double Turn = 0;
        double Heading = 0;
        boolean fieldCentricMode = true;
        double fieldCentricTimer = 0;
        boolean resetLast = false;
        boolean revReq = false;
        boolean shootReq = false;

        //pinpoint.setOffsets(100, -25, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.setHeading(0, AngleUnit.DEGREES);
        pinpoint.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.left_bumper) {
                if (!resetLast) {
                    //imu.resetYaw();
                    pinpoint.setHeading(0, AngleUnit.DEGREES);
                    resetLast = true;
                }
            } else { resetLast = false; }
            pinpoint.update();

            double robotYaw = pinpoint.getHeading(AngleUnit.DEGREES);
            Heading = Math.toRadians(pinpoint.getPosition().getHeading(AngleUnit.DEGREES) + Constants.HeadingOffset);


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

            double rawForward = gamepad1.left_stick_y; // FTC joystick forward is negative
            double rawStrafe = gamepad1.left_stick_x;
            Turn = gamepad1.right_stick_x;

            //Calculate base movement values
            if (fieldCentricMode) {
                telemetry.addData("Drive Mode:", "Field Centric");

                double sinHeading = Math.sin(Heading); // Pinpoint heading is CW+, invert for standard CCW math
                double cosHeading = Math.cos(Heading);

                // Rotate the driver input vector so it is field-centric
                Forward = rawStrafe * sinHeading + -rawForward * cosHeading;
                Strafe = rawStrafe * cosHeading - -rawForward * sinHeading;
            } else {
                telemetry.addData("Drive Mode", "Robot Centric");
                Forward = -rawForward;
                Strafe = rawStrafe;
            }

            //Apply Brakes
            if (gamepad1.right_bumper) {
                Forward *= Constants.brake;
                Strafe *= Constants.brake;
                Turn *= Constants.brake;
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

            if (gamepad2.a)
            {
                intake.setPower(-1);
            }
            else
            {
                intake.setPower(0);
            }

         //   shootReq = gamepad2.left_bumper;
          //  revReq = gamepad2.right_bumper;

         /*   AutoFunctions.runShooterBio(ShFront,
                    intake,
                    2000,
                    getRuntime(),
                    shootReq,
                    revReq); */

            if (gamepad2.left_bumper)
            {
                ShFront.setPower(-0.57);
            } else
            {
                ShFront.setPower(0);
            }
            telemetry.addData("Heading", Heading);

        }
    }
}

