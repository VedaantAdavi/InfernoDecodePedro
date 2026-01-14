package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.jumpypants.murphy.tasks.SequentialTask;
import com.jumpypants.murphy.tasks.Task;
import com.jumpypants.murphy.tasks.WaitTask;
import com.jumpypants.murphy.util.RobotContext;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subSystems.Intake;
import org.firstinspires.ftc.teamcode.subSystems.Shooter;
import org.firstinspires.ftc.teamcode.subSystems.Transfer;
import org.firstinspires.ftc.teamcode.subSystems.Turret;

import java.util.ArrayList;
import java.util.List;

@Configurable
@Autonomous(name="(AUTO) Auton", group="Main")
public class MainAutonFar extends LinearOpMode {
    public enum Alliance {
        RED,
        BLUE
    }

    public static Alliance alliance = Alliance.BLUE;

    private final Pose START_POSE = new Pose(57, 8.5, 0.5 * Math.PI);

    private final List<Path> paths = new ArrayList<>();

    @Override
    public void runOpMode() {
        MyRobot robotContext = new MyRobot(
                hardwareMap,
                telemetry,
                gamepad1,
                gamepad2,
                new Intake(hardwareMap),
                new Shooter(hardwareMap),
                new Transfer(hardwareMap),
                new Turret(hardwareMap)
        );

        MyRobot.follower = Constants.createFollower(hardwareMap);

        Follower follower = MyRobot.follower;

        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        while (opModeInInit()){
            if (gamepad2.left_bumper){
                alliance = Alliance.BLUE;
            } else if (gamepad2.right_bumper) {
                alliance = Alliance.RED;
            }

            telemetry.addLine("Alliance: " + alliance.name());

            telemetry.update();
        }

        robotContext.TURRET.resetEncoder();

        Pose startingPose = mirrorForAlliance(START_POSE.getX(), START_POSE.getY(), START_POSE.getHeading());
        follower.setStartingPose(startingPose);

        double targetX = (alliance == Alliance.BLUE) ? 14 : 135;
        double targetY = 135;

        follower.update();
        follower.startTeleopDrive(true);

        buildPaths();

        Task mainTask = new SequentialTask(robotContext,
                new FollowPathTask(robotContext, follower, paths.get(0)),
                new WaitTask(robotContext, 1),
                robotContext.INTAKE.new SetIntakePower(robotContext, 1),
                robotContext.TRANSFER.new SendThreeTask(robotContext),
                new FollowPathTask(robotContext, follower, paths.get(1)),
                new FollowPathTask(robotContext, follower, paths.get(2)),
                robotContext.TRANSFER.new SendThreeTask(robotContext),
                new FollowPathTask(robotContext, follower, paths.get(3)),
                new WaitTask(robotContext, 0.5),
                new FollowPathTask(robotContext, follower, paths.get(4)),
                robotContext.TRANSFER.new SendThreeTask(robotContext)
        );

        while (opModeIsActive()){
            follower.update();

            Pose currentPose = follower.getPose();

            robotContext.TURRET.setRotation(Turret.calculateGoalRotation(currentPose.getX(), currentPose.getY(), currentPose.getHeading(), targetX, targetY));
            robotContext.TURRET.updatePID();
            robotContext.SHOOTER.updatePID();

            double d = Math.pow(currentPose.getX() - targetX, 2) + Math.pow(currentPose.getY() - targetY, 2);
            robotContext.SHOOTER.setHoodByDistance(d);
            robotContext.SHOOTER.setVelByDistance(d);

            mainTask.step();
        }
    }

    private void buildPaths() {
        addLinePath(61, 22, 0.5 * Math.PI);        // Preload shoot
        addBezierPathWithTangent(15, 36,                   // First intake
                62, 36,
                37, 36
        );
        addLinePathConstant(61, 22, Math.PI);        // Second shoot
        addBezierPath(23, 10.5, (double) 10/9 * Math.PI,                  // Second intake (corner)
                58, 9.5
        );
        addLinePath(61, 22, Math.PI);        // Third shoot
    }

    private Pose mirrorForAlliance(double x, double y, double heading) {
        if (alliance == Alliance.BLUE) {
            return new Pose(x, y, heading);
        } else {
            return new Pose(144 - x, y, Math.PI - heading);
        }
    }

    private void addLinePath(double x, double y, double heading) {
        Pose start = getLastPose();
        Pose end = mirrorForAlliance(x, y, heading);

        Path path = new Path(new BezierLine(start, end));
        path.setLinearHeadingInterpolation(start.getHeading(), end.getHeading());
        paths.add(path);
    }

    private void addLinePathConstant(double x, double y, double heading) {
        Pose start = getLastPose();
        Pose end = mirrorForAlliance(x, y, heading);

        Path path = new Path(new BezierLine(start, end));
        path.setConstantHeadingInterpolation(end.getHeading());
        paths.add(path);
    }

    private void addBezierPath(double x, double y, double heading, double... controlPoints) {
        Pose start = getLastPose();
        Pose end = mirrorForAlliance(x, y, heading);
        Pose[] controlPoses = buildControlPoses(start, end, controlPoints);

        Path path = new Path(new BezierCurve(controlPoses));
        path.setLinearHeadingInterpolation(start.getHeading(), end.getHeading());
        paths.add(path);
    }

    private void addBezierPathWithTangent(double x, double y, double... controlPoints) {
        Pose start = getLastPose();
        Pose end = mirrorForAlliance(x, y, 0);
        Pose[] controlPoses = buildControlPoses(start, end, controlPoints);

        Path path = new Path(new BezierCurve(controlPoses));
        path.setTangentHeadingInterpolation();
        paths.add(path);
    }

    private Pose getLastPose() {
        return paths.isEmpty() ?
            mirrorForAlliance(START_POSE.getX(), START_POSE.getY(), START_POSE.getHeading()) :
            paths.get(paths.size() - 1).getLastControlPoint();
    }

    private Pose[] buildControlPoses(Pose start, Pose end, double... controlPoints) {
        Pose[] controlPoses = new Pose[controlPoints.length / 2 + 2];
        controlPoses[0] = start;

        for (int i = 0; i < controlPoints.length; i += 2) {
            controlPoses[i / 2 + 1] = mirrorForAlliance(controlPoints[i], controlPoints[i + 1], 0);
        }

        controlPoses[controlPoses.length - 1] = end;
        return controlPoses;
    }

    private static class FollowPathTask extends Task {
        private final Follower follower;
        private final Path path;

        public FollowPathTask(RobotContext robotContext, Follower follower, Path path) {
            super(robotContext);
            this.follower = follower;
            this.path = path;
        }

        @Override
        protected void initialize(RobotContext robotContext) {
            follower.followPath(path);
        }

        @Override
        protected boolean run(RobotContext robotContext) {
            return follower.isBusy();
        }
    }
}