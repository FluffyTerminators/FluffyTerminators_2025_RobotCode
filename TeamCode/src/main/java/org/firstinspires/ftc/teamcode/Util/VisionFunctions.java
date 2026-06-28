package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Util.Constants;

import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class VisionFunctions {

    private static Limelight3A Limelight;
    private static FiducialResult LastGoodTag;
    private static double LastGoodTime;
    private static DoubleSupplier runTime;

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

    public static double getDistance()
    {
        if (goodTag()){
            double x = LastGoodTag.getRobotPoseTargetSpace().getPosition().x;
            double z = LastGoodTag.getRobotPoseTargetSpace().getPosition().z;
            return Math.sqrt((x * x) + (z * z));
        }
        else
        {
            return -1.0;
        }
    }

    public static double getAngle()
    {
        if (goodTag())
        {
            return LastGoodTag.getTargetXDegrees();
        } else {
            return 0;
        }
    }
}
