package org.firstinspires.ftc.teamcode.Teleop;

//import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Util.GoBildaPinpointDriver;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
//import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
//import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

//import static com.qualcomm.robotcore.util.TypeConversion.byteArrayToInt;


//Download Missing Files


@TeleOp(name = "Drive Test")
public class DriveTest extends LinearOpMode {

   DcMotor bLDrive;
   DcMotor bRDrive;
   Blinker control_Hub;
   DcMotor fLDrive;
  DcMotor fRDrive;
  GoBildaPinpointDriver odo;
  IMU imu;
  CRServo Intake_Transfer_Servo_1;//, Intake_Transfer_Servo_2,Spindexer_Servo;
  DcMotor Intake;

  public void runOpMode()
  {
    bLDrive = hardwareMap.get(DcMotor.class, "BLDrive");
    bRDrive = hardwareMap.get(DcMotor.class, "BRDrive");
    fLDrive = hardwareMap.get(DcMotor.class, "FLDrive");
    fRDrive = hardwareMap.get(DcMotor.class, "FRDrive");
    imu = hardwareMap.get(IMU.class, "imu");
    odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
    Intake_Transfer_Servo_1 = hardwareMap.get(CRServo.class, "ITServo_1");
    Intake = hardwareMap.get(DcMotor.class, "Intake");
   // Intake_Transfer_Servo_2 = hardwareMap.get(CRServo.class,"ITServo_2");
   // Spindexer_Servo = hardwareMap.get(CRServo.class,"Spindexer_Servo");

    fLDrive.setDirection(DcMotorSimple.Direction.REVERSE);
    bLDrive.setDirection(DcMotorSimple.Direction.REVERSE);
    fRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    bRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    fLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    bLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    imu.initialize(new IMU.Parameters((ImuOrientationOnRobot) new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.DOWN, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT)));

    double Forward = 0;
    double Strafe = 0;
    double Turn = 0;
    double MotorPower = 0;
    double Heading = 0;
    double HeadingOffset = 0;

    odo.setOffsets(0, 0, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
    odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
    odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);


    telemetry.addData("Status", "Initialized");
    telemetry.update();

    waitForStart();
    while (opModeIsActive()) {
      telemetry.addData("Status", "Running");

      odo.update();
      telemetry.addData("Heading Scalar", odo.getYawScalar());

      Heading = Math.toRadians(odo.getPosition().getHeading(AngleUnit.DEGREES) + HeadingOffset);
      telemetry.addData("Heading", Math.toDegrees(Heading));
      Forward = ((Math.cos(Heading) * gamepad2.left_stick_y) + (Math.sin(Heading) * gamepad2.left_stick_x));
      Strafe = ((Math.sin(Heading) * gamepad2.left_stick_y) - (Math.cos(Heading) * gamepad2.left_stick_x));
      Turn = -gamepad2.right_stick_x;

      if (gamepad2.b) {
        odo.recalibrateIMU(); //recalibrates the IMU without resetting position
      }


      if (gamepad2.right_bumper) {
        Forward /= 2;
        Strafe /= 2;
        Turn /= 2;
      }

      if (gamepad1.a)
      {
        Intake.setPower(1);
      }
      else
      {
        Intake.setPower(0);
      }

     /* if (gamepad2.y)
      {
        Spindexer_Servo.setPower(0.5);
      }
      else
      {
        Spindexer_Servo.setPower(0);
      } */
      if (gamepad2.left_bumper) {
        imu.initialize(new IMU.Parameters((ImuOrientationOnRobot) new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT)));
        imu.resetYaw();
        odo.resetPosAndIMU(); //resets the position to 0 and recalibrates the IMU
        odo.setHeading(0, AngleUnit.DEGREES);
        odo.update();
      }

      //odo.update(GoBildaPinpointDriver.ReadData.ONLY_UPDATE_HEADING);
      telemetry.addData("PinPoint Status", odo.getDeviceStatus());

      double denominator = Math.max(Math.abs(Forward) + Math.abs(Strafe) + Math.abs(Turn), 1);
      MotorPower = (Forward + Strafe - Turn) / denominator;
      fRDrive.setPower(MotorPower);
      MotorPower = (Forward - Strafe + Turn) / denominator;
      fLDrive.setPower(MotorPower);
      MotorPower = (Forward - Strafe - Turn) / denominator;
      bRDrive.setPower(MotorPower);
      MotorPower = (Forward + Strafe + Turn) / denominator;
      bLDrive.setPower(MotorPower);
      telemetry.update();
    }
  }
}
