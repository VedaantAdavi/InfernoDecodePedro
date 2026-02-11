package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class LimelightLocalizer {
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
        LLResult result = this.limelight.getLatestResult();
        if (!(result != null && result.isValid())) return null;
        return result.getBotpose();
    }

    public void setPipeline(int pipeline) {
        this.limelight.pipelineSwitch(pipeline);
    }

    public Limelight3A getLimelight() {
        return this.limelight;
    }
}
