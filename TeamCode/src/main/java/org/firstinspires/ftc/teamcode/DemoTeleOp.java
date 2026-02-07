package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.jumpypants.murphy.states.StateMachine;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robotStates.IntakingState;
import org.firstinspires.ftc.teamcode.robotStates.ShootingState;
import org.firstinspires.ftc.teamcode.subSystems.Intake;
import org.firstinspires.ftc.teamcode.subSystems.Shooter;
import org.firstinspires.ftc.teamcode.subSystems.Transfer;
import org.firstinspires.ftc.teamcode.subSystems.Turret;

@Configurable
@TeleOp(name="Demo Tele-Op", group="Main")
public class DemoTeleOp extends LinearOpMode {
    StateMachine stateMachine;

    public static double TARGET_X, TARGET_Y;

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
        stateMachine = new StateMachine(new ShootingState(robotContext), robotContext);
        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        waitForStart();

        MyRobot.follower = Constants.createFollower(hardwareMap);

        Follower follower = MyRobot.follower;

        Pose startingPose = new Pose(0, 0, Math.PI / 2);
        follower.setStartingPose(startingPose);

        TARGET_X = 0;
        TARGET_Y = 144;

        follower.update();

        follower.startTeleopDrive(true);

        while (opModeIsActive()){
            stateMachine.step();

            follower.update();

            Pose currentPose = follower.getPose();

            robotContext.TURRET.setRotation(Turret.calculateGoalRotation( currentPose.getX(), currentPose.getY(), currentPose.getHeading(), TARGET_X, TARGET_Y));
            robotContext.TURRET.updatePID();
            robotContext.SHOOTER.updatePID();

            double d = getDistanceToTarget();
            robotContext.SHOOTER.setHoodByDistance(d, telemetryM);

            if (gamepad1.dpad_right) {
                robotContext.TURRET.incrementAngleOffset(0.01);
            }
            if (gamepad1.dpad_left) {
                robotContext.TURRET.incrementAngleOffset(-0.01);
            }
            if(gamepad1.dpad_down) {
                robotContext.TURRET.resetEncoder();
            }
            if (gamepad1.triangle) {
                follower.setPose(startingPose);
            }

            follower.setTeleOpDrive(
                    Math.pow(gamepad1.left_stick_x, 3),
                    Math.pow(-gamepad1.left_stick_y, 3),
                    Math.pow(-gamepad1.right_stick_x, 3),
                    false // field Centric
            );

            telemetryM.debug("x pos", currentPose.getX());
            telemetryM.debug("y pos", currentPose.getY());
            telemetryM.debug("heading pos", currentPose.getHeading());
            telemetryM.debug("Distance from goal", d);
            telemetryM.addData("Current Velocity", robotContext.SHOOTER.getVelocity());
            telemetryM.addData("Target Velocity", robotContext.SHOOTER.getTargetVelocity());
            telemetryM.update();

            telemetry.addData("Current Velocity", robotContext.SHOOTER.getVelocity());
            telemetry.addData("Target Velocity", robotContext.SHOOTER.getTargetVelocity());

            telemetry.update();
        }
    }

    public double getDistanceToTarget() {
        return Math.sqrt(Math.pow(MyRobot.follower.getPose().getX() - TARGET_X, 2) + Math.pow(MyRobot.follower.getPose().getY() - TARGET_Y, 2));
    }
}