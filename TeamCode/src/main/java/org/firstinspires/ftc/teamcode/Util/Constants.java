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
            .rightFrontMotorName("fRDrive")
            .rightRearMotorName("bRDrive")
            .leftRearMotorName("bLDrive")
            .leftFrontMotorName("fLDrive")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-5)
            .strafePodX(0.5)
            .distanceUnit(DistanceUnit.CM)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static class PEDROConstants
    {
        public static FollowerConstants followerConstants = new FollowerConstants();

        public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

        public static Follower createFollower (HardwareMap hardwareMap)
        {
            return new FollowerBuilder(followerConstants, hardwareMap)
                    .pathConstraints(pathConstraints)
                    .mecanumDrivetrain(driveConstants)
                    .build();

        }
    }

    public static final double HeadingOffset = 0;
    public static final double spindexerPower = 1;
    public static final double shooterPower = 1;
    public static final double brake = 2;
    public static final double flapUp = 0.55;
    public static final double flapDeploy = 0.5;
    public static final double flapDown = 0.45;

    public static class HardwareMappings
    {
        // Hubs
        public static Blinker control_Hub = hardwareMap.get(Blinker.class, "control_Hub");
        public static Blinker expansion_Hub_2 = hardwareMap.get(Blinker.class, "expansion_Hub_2");

        // Drive Motors
        public static DcMotor bLDrive = hardwareMap.get(DcMotor.class, "BLDrive");
        public static DcMotor bRDrive = hardwareMap.get(DcMotor.class, "BRDrive");
        public static DcMotor fLDrive = hardwareMap.get(DcMotor.class, "FLDrive");
        public static DcMotor fRDrive = hardwareMap.get(DcMotor.class, "FRDrive");

        // Mechanism Motors
        public static DcMotor Intake  = hardwareMap.get(DcMotor.class, "Intake");
        public static DcMotor Shooter = hardwareMap.get(DcMotor.class, "Shooter");

        // Internal Motion Units
        public static IMU imu                        = hardwareMap.get(IMU.class,                   "imu");
        public static GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Servos
        public static CRServo IntakeTransferServo1 = hardwareMap.get(CRServo.class, "ITServo_1");
        public static CRServo IntakeTransferServo2 = hardwareMap.get(CRServo.class, "ITServo_2");
        public static CRServo SpindxerServo        = hardwareMap.get(CRServo.class, "Spindexer_Servo");
        public static Servo Flap                   = hardwareMap.get(Servo.class,   "Flap");

        // Colour Sensors
        public static NormalizedColorSensor SpindexerSensor1 = hardwareMap.get(NormalizedColorSensor.class, "spindexer_colour_1");
        public static NormalizedColorSensor SpindexerSensor2 = hardwareMap.get(NormalizedColorSensor.class, "spindexer_colour_2");


        public static FollowerConstants followerConstants = new FollowerConstants()
                .mass(9.35);

    }
}
// things to make the stupid code commit. write whatever u want here, so long as it changes the code
// for example, i'm gonna yap about keyboards here just so i can commit
// actually im not going to because that's gonna take too long lol
// hrmm okay here we go agian