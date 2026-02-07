package org.firstinspires.ftc.teamcode.subSystems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.jumpypants.murphy.util.RobotContext;
import com.jumpypants.murphy.tasks.Task;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.InterpolatedLUT;
import org.firstinspires.ftc.teamcode.MyRobot;

import java.util.ArrayList;
import java.util.Arrays;

@Configurable
public class Shooter {
    public static ArrayList<Double> DISTANCE_POINTS = new ArrayList<>(Arrays.asList(33.0, 40.0, 50.0, 60.0, 70.0, 80.0, 111.0));
    public static ArrayList<Double> HOOD_ANGLE_POINTS = new ArrayList<>(Arrays.asList(0.45, 0.45, 0.4, 0.4, 0.4, 0.35, 0.15));
    public static ArrayList<Double> VELOCITY_POINTS = new ArrayList<>(Arrays.asList(0.62, 0.62, 0.65, 0.66, 0.67, 0.67, 0.8));

    public static double IDLE_VEL = 0.3;

    public static double P = 0.15;
    public static double I = 0.0;
    public static double D = 0.0;

    public static double V = 1.255;
    public static double S = 0;

    private final Servo HOOD_SERVO;
    public final double HOOD_MAX_POS = 0.45;
    public final double HOOD_MIN_POS = 0;
    private final Motor LEFT_WHEEL;
    private final Motor RIGHT_WHEEL;

    private double targetVelocity = 0.0;

    private double hoodOffset = 0;
    private double shooterOffset = 0;

    public Shooter(HardwareMap hardwareMap) {
        HOOD_SERVO = hardwareMap.get(Servo.class, "Hood");

        LEFT_WHEEL = new Motor(hardwareMap, "LeftShooter");
        RIGHT_WHEEL = new Motor(hardwareMap, "RightShooter");

        LEFT_WHEEL.setRunMode(Motor.RunMode.VelocityControl);
        RIGHT_WHEEL.setRunMode(Motor.RunMode.VelocityControl);

        LEFT_WHEEL.setVeloCoefficients(P, I, D);
        RIGHT_WHEEL.setVeloCoefficients(P, I, D);
        LEFT_WHEEL.setFeedforwardCoefficients(S, V);
        RIGHT_WHEEL.setFeedforwardCoefficients(S, V);

        RIGHT_WHEEL.setInverted(true);

    }

    public void setVel(double vel) {
        targetVelocity = vel+shooterOffset;
    }

    private double calcHoodPos (double d) {
        return InterpolatedLUT.get(DISTANCE_POINTS, HOOD_ANGLE_POINTS, d);
    }

    private double calcVelocity (double d) {
        return InterpolatedLUT.get(DISTANCE_POINTS, VELOCITY_POINTS, d);
    }

    public void setHoodByDistance(double distance) {
        setHoodPosition(calcHoodPos(distance));
    }

    public void setHoodByDistance(double distance, TelemetryManager telemetry) {
        setHoodPosition(calcHoodPos(distance), telemetry);
    }

    public void setVelByDistance(double distance) {
        setVel(calcVelocity(distance));
    }

    public void setHoodPosition(double position) {
        position += hoodOffset;
        double clampedPosition = Math.max(HOOD_MIN_POS, Math.min(HOOD_MAX_POS, position));
        HOOD_SERVO.setPosition(clampedPosition);
    }
    public void setHoodPosition(double position, TelemetryManager telemetry) {
        position += hoodOffset;
        double clampedPosition = Math.max(HOOD_MIN_POS, Math.min(HOOD_MAX_POS, position));
        telemetry.debug("target hood pos", clampedPosition);
        HOOD_SERVO.setPosition(clampedPosition);
    }

    public void incrementHoodOffset(double i) {
        hoodOffset += i;
    }
    public void incrementShooterOffset(double i){
        shooterOffset +=i;
    }

    public void setHoodOffset(double o) {
        hoodOffset = o;
    }
    public void setShooterOffset(double o){shooterOffset = o;}

    public void updatePID() {
        LEFT_WHEEL.setVeloCoefficients(P, I, D);
        RIGHT_WHEEL.setVeloCoefficients(P, I, D);

        LEFT_WHEEL.setFeedforwardCoefficients(S, V);
        RIGHT_WHEEL.setFeedforwardCoefficients(S, V);

        LEFT_WHEEL.set(targetVelocity+shooterOffset);
        RIGHT_WHEEL.set(targetVelocity+shooterOffset);
    }

    public double getVelocity() {
        double maxTicksPerSec = (6000.0 * 28.0) / 60.0;
        double currentVelocityTicksPerSecLEFT = LEFT_WHEEL.getCorrectedVelocity();
        double currentVelocityTicksPerSecRIGHT = RIGHT_WHEEL.getCorrectedVelocity();

        double normalizedVelocity = (currentVelocityTicksPerSecLEFT / maxTicksPerSec) + (currentVelocityTicksPerSecRIGHT / maxTicksPerSec);
        normalizedVelocity /= 2;

        return normalizedVelocity;
    }

    public double getTargetVelocity() {
        return targetVelocity+shooterOffset;
    }

    public class setVelocity extends Task{

        public setVelocity(RobotContext robotContext, double vel){
            super(robotContext);
            setVel(vel);
        }

        @Override
        protected void initialize(RobotContext robotContext) {
        }

        @Override
        protected boolean run(RobotContext robotContext) {
            return false;
        }
    }

    public class RunOuttakeTask extends Task {

        private final double POWER;

        public RunOuttakeTask(MyRobot robotContext, double power) {
            super(robotContext);
            POWER = power;
        }

        @Override
        protected void initialize(RobotContext robotContext) {
            LEFT_WHEEL.set(POWER);
            RIGHT_WHEEL.set(POWER);
        }

        @Override
        protected boolean run(RobotContext robotContext) {
            return false;
        }
    }
}
