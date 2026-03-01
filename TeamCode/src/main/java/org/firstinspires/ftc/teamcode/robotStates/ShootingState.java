package org.firstinspires.ftc.teamcode.robotStates;

import com.jumpypants.murphy.states.State;
import com.jumpypants.murphy.tasks.QueueTask;
import com.jumpypants.murphy.tasks.Task;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.compOpmode.MainTeleOp;
import org.firstinspires.ftc.teamcode.MyRobot;

public class ShootingState implements State {

    private final MyRobot robotContext;
//    private final Task mainTask;

    private final QueueTask transferTask;
    private final Task intakeTask;

    public ShootingState(MyRobot robotContext) {
        this.robotContext = robotContext;
        transferTask = robotContext.TRANSFER.new TransferTask(robotContext);
        intakeTask = robotContext.INTAKE.new ManualRunIntakeMotor(robotContext);


//        mainTask = new SequentialTask(robotContext,
//                new ParallelTask(robotContext, true,
//                        robotContext.TRANSFER.new TransferTask(robotContext),
//                        robotContext.INTAKE.new ManualRunIntakeMotor(robotContext)
//                )
//        );
    }

    @Override
    public State step() {
        robotContext.GAMEPAD1.stopRumble();
        robotContext.GAMEPAD2.stopRumble();
        robotContext.GAMEPAD1.rumble(0.1, 0.1, 200);
        robotContext.GAMEPAD2.rumble(0.1, 0.1, 200);

        Pose currentPose = MyRobot.follower.getPose();

        double d = Math.sqrt(Math.pow(currentPose.getX() - MainTeleOp.TARGET_X, 2) + Math.pow(currentPose.getY() - MainTeleOp.TARGET_Y, 2));

        robotContext.SHOOTER.setVelByDistance(d);

        transferTask.step();

        if (!transferTask.isEmpty()){
            return this;
        } else if (intakeTask.step()) {
            return this;
        } else {
            return new IntakingState(robotContext);
        }

//        if (mainTask.step()) {
//            return this;
//        }
    }


    @Override
    public String getName() {
        return "Shooting";
    }
}
