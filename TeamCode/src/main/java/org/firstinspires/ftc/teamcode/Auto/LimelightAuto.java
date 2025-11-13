package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.Util.Constants.flapDeploy;
import static org.firstinspires.ftc.teamcode.Util.Constants.flapUp;
import static org.firstinspires.ftc.teamcode.Util.Constants.spindexerBWD;
import static org.firstinspires.ftc.teamcode.Util.Constants.spindexerFWD;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Util.Constants;
import org.firstinspires.ftc.teamcode.Util.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.*;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.Collections;
import java.util.List;
@Disabled
@Autonomous(name = "Limelight")
public class LimelightAuto extends OpMode {

  public Limelight3A limelight;

  @Override
  public void init()
  {
    limelight = hardwareMap.get(Limelight3A.class, "limelight");
    limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
    limelight.start(); // This tells Limelight to start looking!
  }

  @Override
  public void loop() {
    LLResult result = limelight.getLatestResult();
    if (result != null && result.isValid()) {
      double tx = result.getTx(); // How far left or right the target is (degrees)
      double ty = result.getTy(); // How far up or down the target is (degrees)
      double ta = result.getTa(); // How big the target looks (0%-100% of the image)

      telemetry.addData("Target X", tx);
      telemetry.addData("Target Y", ty);
      telemetry.addData("Target Area", ta);
    } else {
      telemetry.addData("Limelight", "No Targets");
    }
  }
  public enum Motif
  {
    FIRST_GREEN,
    MIDDLE_GREEN,
    LAST_GREEN
  }
}
