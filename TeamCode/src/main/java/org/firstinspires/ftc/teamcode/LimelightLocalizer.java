package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@Configurable
public class LimelightLocalizer {
    public static double X_NUDGE = 0;
    public static double Y_NUDGE = 0;
    public static double X_SCALE = 1.01;
    public static double Y_SCALE = 1;

    private final Limelight3A limelight;
    public LimelightLocalizer(Limelight3A limelight) {
        this.limelight = limelight;
    }

    public void start() {
        this.limelight.start();
    }

    public void stop() {
        this.limelight.stop();
    }

    public Pose3D getPose() {
        LLResult result = limelight.getLatestResult();
        if (!(result != null && result.isValid())) return null;
        return result.getBotpose();
    }

    public Pose getPosePedro() {
        Pose3D llOutput = getPose();

        if (llOutput == null) return null;

        Pose limelightPose = new Pose(llOutput.getPosition().x * 39.3701, llOutput.getPosition().y * 39.3701, llOutput.getOrientation().getYaw(AngleUnit.RADIANS), FTCCoordinates.INSTANCE);

        double heading = limelightPose.getHeading();
        heading -= Math.PI / 2;
        heading = heading < 0 ? heading + Math.PI * 2 : heading;

        double x = limelightPose.getY() + 72;
        double y = -limelightPose.getX() + 72;

        y *= Y_SCALE;
        y += Y_NUDGE;

        x *= X_SCALE;
        x += X_NUDGE;

        return new Pose(x, y, heading);
    }

    public void setPipeline(int pipeline) {
        this.limelight.pipelineSwitch(pipeline);
    }

    public Limelight3A getLimelight() {
        return this.limelight;
    }
}
