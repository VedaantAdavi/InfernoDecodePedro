package org.firstinspires.ftc.teamcode.debug;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subSystems.Shooter;
import org.firstinspires.ftc.teamcode.subSystems.Turret;
import org.firstinspires.ftc.teamcode.subSystems.Transfer;

@Configurable

@TeleOp(name="Debug - Shooter PID Tuner", group="Debug")
public class ShooterPIDTuner extends LinearOpMode {
    public static double TARGET_VELOCITY = 0.0;

    @Override
    public void runOpMode() {
        Shooter shooter = new Shooter(hardwareMap);

        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        waitForStart();

        while (opModeIsActive()) {
            shooter.setVel(TARGET_VELOCITY);
            shooter.updatePID();

            telemetryM.addData("Current Velocity", shooter.getVelocity());
            telemetryM.addData("Target Velocity", TARGET_VELOCITY);


            telemetryM.update();
        }
    }
}