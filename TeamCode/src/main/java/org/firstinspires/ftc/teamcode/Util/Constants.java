package org.firstinspires.ftc.teamcode.Util;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.GoBildaOdometryPods;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import static com.qualcomm.robotcore.util.TypeConversion.byteArrayToInt;
import org.firstinspires.ftc.teamcode.Util.Constants;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.GoBildaOdometryPods.*;


public class Constants
{

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("FRDrive")
            .rightRearMotorName("BRDrive")
            .leftRearMotorName("BLDrive")
            .leftFrontMotorName("FLDrive")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(54.009306179256896)
            .yVelocity(40.09489380843996);


    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-25)
            .strafePodX(100)
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static class PEDROConstants
    {
        public static FollowerConstants followerConstants = new FollowerConstants()
                .forwardZeroPowerAcceleration(55.50120432611453)
                .lateralZeroPowerAcceleration(84.22805501815317);

        public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

        public static Follower createFollower (HardwareMap hardwareMap)
        {
            return new FollowerBuilder(followerConstants, hardwareMap)
                    .pathConstraints(pathConstraints)
                    .pinpointLocalizer(localizerConstants)
                    .mecanumDrivetrain(driveConstants)
                    /* other builder steps */
                    .build();

        }
    }

    public static final double HeadingOffset = 0;
    public static final double spindexerFWD = 1;
    public static final double spindexerBWD = -1;
    public static final double shooterPower = 1;
    public static final double brake = 2;
    public static final int LLPipeline = 7;

    public static final double High_Overrride_Speed = 900;
    public static final double Low_Override_Speed = 740;
    public static final double Shooter_Speed_Tolerance = 40;

    public static class HardwareMappings {
        // Hubs
        public static Blinker control_Hub = hardwareMap.get(Blinker.class, "control_Hub");
        public static Blinker expansion_Hub_2 = hardwareMap.get(Blinker.class, "expansion_Hub_2");

        // Drive Motors
        public static DcMotor bLDrive = hardwareMap.get(DcMotor.class, "BLDrive");
        public static DcMotor bRDrive = hardwareMap.get(DcMotor.class, "BRDrive");
        public static DcMotor fLDrive = hardwareMap.get(DcMotor.class, "FLDrive");
        public static DcMotor fRDrive = hardwareMap.get(DcMotor.class, "FRDrive");

        // Mechanism Motors
        public static DcMotor Intake = hardwareMap.get(DcMotor.class, "Intake");
        public static DcMotor Shooter = hardwareMap.get(DcMotor.class, "Shooter");

        // Internal Motion Units
        public static IMU imu = hardwareMap.get(IMU.class, "imu");
        public static GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Servos
        public static CRServo IntakeTransferServo1 = hardwareMap.get(CRServo.class, "ITServo_1");
        public static CRServo IntakeTransferServo2 = hardwareMap.get(CRServo.class, "ITServo_2");
        public static CRServo SpindxerServo = hardwareMap.get(CRServo.class, "Spindexer_Servo");
        public static Servo Flap = hardwareMap.get(Servo.class, "Flap");

        // Colour Sensors
        public static NormalizedColorSensor SpindexerSensor1 = hardwareMap.get(NormalizedColorSensor.class, "spindexer_colour_1");
        public static NormalizedColorSensor SpindexerSensor2 = hardwareMap.get(NormalizedColorSensor.class, "spindexer_colour_2");
    }

    public static class ShooterCal
    {
        private static final double[] distance     = {0.64, 0.82, 1.20, 1.36, 1.58, 1.85, 2.00, 2.86, 3.00};
        private static final double[] shooterTicks = {540, 540, 560, 580, 600, 620, 640, 740, 760};

        /**
         * interpolate(x) uses the distance from the target to automatically return the required shooter speed in motor velocity
         * @param x = distance in METERS
         * */
        public static double interpolate(double x)
        {
            //Find the bracketing points
           int i = 0;
           while (i < distance.length - 1 && distance[i + 1] < x)
           {
               i++;
           }

           //edge case handling
            if (x < distance[0]) {return shooterTicks[0];}
            if (x > distance[distance.length - 1]) {return shooterTicks[distance.length - 1];}

            double x1 = distance[i];
            double y1 = shooterTicks[i];
            double x2 = distance[i + 1];
            double y2 = shooterTicks[i + 1];

            return y1 + (x - x1) * (y2 - y1) / (x2 - x1);
        }
    }
}
