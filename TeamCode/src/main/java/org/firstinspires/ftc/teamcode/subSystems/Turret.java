package org.firstinspires.ftc.teamcode.subSystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class Turret {
    private final Motor turretMotor;
    private final Motor encoderRefMotor;
    private final PIDController PID;

    public static double P = 0.95;
    public static double I = 0.03;
    public static double D = 0.02;

    public static double MAX_POWER = 1.0;
    public static double MIN_POWER = -1.0;
    public static double MAX_ANGLE = 11.0/18 * Math.PI;
    public static double MIN_ANGLE = -11.0/18 * Math.PI;

    public static final double TICKS_PER_REV = 384.5;
    public static final double GEAR_RATIO = 215.0/40.0;

    private double angleOffset = 0;

    public Turret(HardwareMap hardwareMap) {
        turretMotor = new Motor(hardwareMap, "Turret");
        turretMotor.setRunMode(Motor.RunMode.RawPower);
        turretMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        encoderRefMotor = new Motor(hardwareMap, "BackRight");
        PID = new PIDController(P, I, D);
    }

    /**
     * <p>Calculate the angle needed for the robot's turret to aim to a specific position on the field.<br />Note that <span style="font-style:italic">either degrees or radians can be used</span>, but you have to be consistent.
     * @param cX The robot's current X position
     * @param cY The robot's current Y position
     * @param cR The robot's current rotation
     * @param tX The target's X position
     * @param tY The target's Y position
     * @return Suggested angle that the turret should face
     */
    public static double calculateGoalRotation(double cX, double cY, double cR, double tX, double tY) {
        double theta = Math.atan2(tY - cY, tX - cX);
        return cR - theta;
    }

    /**
     * <p>Estimates the robot's position error (x and y) based on the manually tuned angle offset.</p>
     * <p>This function assumes the distance to the goal is correct and uses the angle offset
     * (which compensates for localization drift) to calculate where the goal would actually be
     * in field coordinates. The difference between the theoretical goal position and the
     * calculated position represents the localization error.</p>
     *
     * @param cX The robot's current X position (from localizer)
     * @param cY The robot's current Y position (from localizer)
     * @param cR The robot's current rotation (from localizer)
     * @param tX The target's actual X position on the field
     * @param tY The target's actual Y position on the field
     * @return A double array [errorX, errorY] representing the estimated position error that should be corrected in the localizer
     */
    public double[] estimatePositionErrorFromAngleOffset (double cX, double cY, double cR, double tX, double tY) {
        // Step 1: Calculate the distance from robot to target
        // We assume this distance is correct (odometry tracks distance better than absolute position)
        double theoreticalDistance = Math.hypot(tX - cX, tY - cY);

        // Step 2: Calculate the field angle from robot to target based on localizer position
        // This is the angle in field coordinates (0 = east, counterclockwise positive)
        double fieldAngleToTarget = Math.atan2(tY - cY, tX - cX);

        // Step 3: Apply the manually tuned angle offset
        // If angleOffset is positive, the turret had to rotate more than expected to hit the target
        // This means the actual angle to the target is different than what the localizer thinks
        double correctedFieldAngle = fieldAngleToTarget - angleOffset;

        // Step 4: Calculate where the goal would "actually" be in field coordinates
        // Using the assumed-correct distance and the corrected angle
        double estimatedGoalX = cX + theoreticalDistance * Math.cos(correctedFieldAngle);
        double estimatedGoalY = cY + theoreticalDistance * Math.sin(correctedFieldAngle);

        // Step 5: The difference between the real target position and estimated position
        // represents the localization error that should be corrected
        double errorX = tX - estimatedGoalX;
        double errorY = tY - estimatedGoalY;

        return new double[] {errorX, errorY};
    }

    public void setRotation(double theta) {
        double normalizedTheta = ((theta % (2 * Math.PI)) + (2 * Math.PI)) % (2 * Math.PI);
        if (normalizedTheta > Math.PI) normalizedTheta -= 2 * Math.PI;
        double clampedTheta = Math.max(Math.min(normalizedTheta, MAX_ANGLE), MIN_ANGLE);
        PID.setSetPoint(clampedTheta + angleOffset);
    }

    public void updatePID() {
        PID.setPID(P, I, D);
        double currentPosition = getCurrentRotation();
        double power = -PID.calculate(currentPosition);
        power = Math.max(Math.min(power, MAX_POWER), MIN_POWER);
        turretMotor.set(power);
    }

    public void incrementAngleOffset(double i) {
        angleOffset += i;
    }

    public void setAngleOffset (double o) {
        angleOffset = o;
    }

    public double getCurrentRotation() {
        return -encoderRefMotor.getCurrentPosition() / TICKS_PER_REV / GEAR_RATIO * 2 * Math.PI;
    }

    public void resetEncoder() {
        encoderRefMotor.resetEncoder();
    }
}
