package org.firstinspires.ftc.teamcode.CleanUp;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;
import org.firstinspires.ftc.teamcode.Util.Constants;

import java.util.Arrays;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Vision {

    private static Limelight3A Limelight;
    private static FiducialResult LastGoodTag;
    private static double LastGoodTime;
    private static DoubleSupplier runTime;

    public static double[] RollingDistance = new double[Constants.rollingDistanceLength];
    private static int RollingIndex;

    private static int lastID;


    public static void init(Limelight3A lime, DoubleSupplier Time)
    {
        Limelight = lime;
        runTime = Time;
        Limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        Limelight.start(); // This tells Limelight to start looking!
        Limelight.pipelineSwitch(Constants.LLPipeline); // Switch to pipeline number 0

    }

    public static void update(double robotYaw)
    {
        Limelight.updateRobotOrientation(robotYaw);
        LLResult result = Limelight.getLatestResult();
        if ((result != null) && (result.isValid()))
        {
            List<FiducialResult> fiducials = result.getFiducialResults();
            for (FiducialResult fiducial : fiducials)
            {
                int id = fiducial.getFiducialId(); // The ID number of the fiducial
                if ((id == 20) || (id == 24))
                {
                    LastGoodTag = fiducial;
                    LastGoodTime = runTime.getAsDouble();
                }
            }
        }
    }

    public static boolean goodTag()
    {
        return (runTime.getAsDouble() - LastGoodTime) < Constants.tagLife;
    }

    public static boolean goodTagNow()
    {
        return (runTime.getAsDouble() - LastGoodTime) < Constants.immediateTagLife;
    }

    public static double getDistance()
    {
        double currentDistance = 0;
        if (goodTag()){
            double AverageDistance = 0.0;
            double x = LastGoodTag.getRobotPoseTargetSpace().getPosition().x;
            double z = LastGoodTag.getRobotPoseTargetSpace().getPosition().z;
            currentDistance = Math.sqrt((x * x) + (z * z));

            if (LastGoodTag.getFiducialId() != lastID){
                Arrays.fill(RollingDistance, currentDistance);
            } else {
                RollingDistance[RollingIndex] = currentDistance;
                RollingIndex ++;
                if (RollingIndex >= RollingDistance.length){
                    RollingIndex = 0;
                }
            }

            lastID = LastGoodTag.getFiducialId();

            for (double distance : RollingDistance){
                AverageDistance += distance;
            }
            AverageDistance /= RollingDistance.length;

            return AverageDistance;
        }
        else
        {
            return -1.0;
        }
    }

    public static double getAngle(boolean immediate)
    {
        if ((goodTagNow() && immediate) || (goodTag() && !immediate)) {
            double offset = LastGoodTag.getTargetXDegrees();
            if (Math.abs(offset) > Constants.headingOffsetDeadZone) {
                if (getDistance() > 2.6) {
                    if (LastGoodTag.getFiducialId() == 20) {
                        offset += Constants.headingBlueDistanceCorrection;
                    } else if (LastGoodTag.getFiducialId() == 24) {
                        offset += Constants.headingRedDistanceCorrection;
                    }
                }
                return offset + Constants.headingOffsetCorrection;
            } else {
                return 0;
            }
        } else {
            return 0;
        }
    }
}
