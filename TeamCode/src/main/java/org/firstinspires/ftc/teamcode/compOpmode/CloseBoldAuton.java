package org.firstinspires.ftc.teamcode.compOpmode;

import com.bylazar.configurables.annotations.Configurable;
import com.jumpypants.murphy.tasks.SequentialTask;
import com.jumpypants.murphy.tasks.Task;
import com.jumpypants.murphy.tasks.WaitTask;
import com.jumpypants.murphy.util.RobotContext;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MyRobot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subSystems.Intake;
import org.firstinspires.ftc.teamcode.subSystems.Shooter;
import org.firstinspires.ftc.teamcode.subSystems.Transfer;
import org.firstinspires.ftc.teamcode.subSystems.Turret;

import java.util.List;


@Configurable
@Autonomous(name="CloseBoldAuton", group="Main")
public class CloseBoldAuton extends LinearOpMode {

    public enum Alliance {
        RED,
        BLUE
    }

    public static Alliance alliance = Alliance.BLUE;
    private MyRobot robotContext;
    Follower follower;

    @Override
    public void runOpMode() throws InterruptedException {
        MyRobot.follower = Constants.createFollower(hardwareMap);
        follower = MyRobot.follower;

        robotContext = new MyRobot(
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

        follower = MyRobot.follower;


//        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        while (opModeInInit()){
            if (gamepad2.left_bumper){
                alliance = Alliance.BLUE;
            } else if (gamepad2.right_bumper) {
                alliance = Alliance.RED;
            }

            telemetry.addLine("Alliance: " + alliance.name());
            telemetry.update();
        }

        Pose startingPose = Paths.mirror(new Pose(27.316, 131.650, Math.toRadians(143)));
        follower.setStartingPose(startingPose);


        robotContext.TURRET.resetEncoder();

        Paths paths = new Paths(follower);

        SequentialTask mainTask = buildMainTask(paths);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        while (opModeIsActive()) {
            mainTask.step();
            follower.update();

            Pose currentPose = follower.getPose();

            Pose target = Paths.mirror(new Pose(9, 140, 0));

            robotContext.TURRET.setRotation(Turret.calculateGoalRotation( currentPose.getX(), currentPose.getY(), currentPose.getHeading(), target.getX(), target.getY()));
            robotContext.TURRET.updatePID();
            robotContext.SHOOTER.updatePID();

            double d = Math.sqrt(Math.pow(currentPose.getX() - target.getX(), 2) + Math.pow(currentPose.getY() - target.getY(), 2));
            robotContext.SHOOTER.setHoodByDistance(d);
            robotContext.SHOOTER.setVelByDistance(d);
        }
    }

    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, GatePath, Path7, Path8;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    mirror(new Pose(24.363, 130.690)),
                                    mirror(new Pose(53.341, 89.224))
                            )
                    ).setLinearHeadingInterpolation(
                            mirrorHeading(Math.toRadians(140)),
                            mirrorHeading(Math.toRadians(188))
                    )
                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    mirror(new Pose(53.341, 89.224)),
                                    mirror(new Pose(20.0, 84.507))
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    mirror(new Pose(20.0, 84.507)),
                                    mirror(new Pose(53.140, 89.047))
                            )
                    ).setConstantHeadingInterpolation(
                            mirrorHeading(Math.toRadians(172))
                    )
                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    mirror(new Pose(53.140, 89.047)),
                                    mirror(new Pose(48, 62.5)),
                                    mirror(new Pose(16.233, 57.814))
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            GatePath = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    mirror(new Pose(18.233, 59.814)),
                                    mirror(new Pose(44.140, 34.581)),
                                    mirror(new Pose(51.581, 82.930)),
                                    mirror(new Pose(21.0, 68.512))
                            )
                    ).setLinearHeadingInterpolation(
                            mirrorHeading(Math.toRadians(180)),
                            mirrorHeading(Math.toRadians(90))
                    )
                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    mirror(new Pose(21.0, 69.512)),
                                    mirror(new Pose(45.476, 67.558)),
                                    mirror(new Pose(53.233, 88.977))
                            )
                    ).setConstantHeadingInterpolation(
                            mirrorHeading(Math.toRadians(90))
                    )
                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    mirror(new Pose(53.233, 88.977)),
                                    mirror(new Pose(79, 24)),
                                    mirror(new Pose(17, 31))
                            )
                    ).setConstantHeadingInterpolation(
                            mirrorHeading(Math.toRadians(180))
                    )
                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    mirror(new Pose(19, 35)),
                                    mirror(new Pose(53.233, 88.977))
                            )
                    ).setLinearHeadingInterpolation(
                            mirrorHeading(Math.toRadians(180)),
                            mirrorHeading(Math.toRadians(90))
                    )
                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    mirror(new Pose(53.233, 88.977)),
                                    mirror(new Pose(33, 81))
                            )
                    ).setConstantHeadingInterpolation(
                            mirrorHeading(Math.toRadians(110))
                    )
                    .build();
        }
        public static Pose mirror(Pose pose) {
            if (alliance == Alliance.RED) {
                return new Pose(144 - pose.getX(), pose.getY(), Math.PI - pose.getHeading());
            }
            return pose;
        }
        private double mirrorHeading(double heading) {
            if (alliance == Alliance.RED) {
                return Math.PI - heading;
            }
            return heading;
        }
    }
    public class goToPath extends Task{
        private final PathChain path;
        public goToPath(PathChain path){
            super(robotContext);
            this.path = path;
        }

        @Override
        public void initialize(RobotContext robotContext){
            follower.followPath(path);
        }

        @Override
        protected boolean run(RobotContext robotContext){
            return follower.isBusy();
        }
    }
    public SequentialTask buildMainTask (Paths paths){
        return new SequentialTask(
                robotContext,

                robotContext.SHOOTER.new RunOuttakeTask(robotContext, 1),
                new goToPath(paths.Path1),
                robotContext.TRANSFER.new SendThreeTask(robotContext),
                robotContext.SHOOTER.new RunOuttakeTask(robotContext, Shooter.IDLE_VEL),

                robotContext.INTAKE.new SetIntakePower(robotContext, 1),
                new goToPath(paths.Path2),

                robotContext.SHOOTER.new RunOuttakeTask(robotContext, 1),
                new goToPath(paths.Path3),
                robotContext.TRANSFER.new SendThreeTask(robotContext),
                robotContext.SHOOTER.new RunOuttakeTask(robotContext, Shooter.IDLE_VEL),

                robotContext.INTAKE.new SetIntakePower(robotContext, 1),
                new goToPath(paths.Path4),

                new goToPath(paths.GatePath),
                new WaitTask(robotContext, 1.6),

                robotContext.SHOOTER.new RunOuttakeTask(robotContext, 1),
                new goToPath(paths.Path5),
                robotContext.TRANSFER.new SendThreeTask(robotContext),
                robotContext.SHOOTER.new RunOuttakeTask(robotContext, Shooter.IDLE_VEL),

                robotContext.INTAKE.new SetIntakePower(robotContext, 1),
                new goToPath(paths.Path6),

                robotContext.SHOOTER.new RunOuttakeTask(robotContext, 1),
                new goToPath(paths.Path7),
                robotContext.TRANSFER.new SendThreeTask(robotContext),
                robotContext.SHOOTER.new RunOuttakeTask(robotContext, Shooter.IDLE_VEL),

                new goToPath(paths.Path8)
        );
    }
}