package org.firstinspires.ftc.teamcode.debug;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subSystems.Turret;

@Configurable
@TeleOp(name="Turret Rotation Debugger", group="Debug")
public class TurretRotationTeleOp extends LinearOpMode {
    public static double cX, cY = 72;
    public static double cR = 0;
    public static double tX, tY = 144;

    @Override
    public void runOpMode() {
        Turret turret = new Turret(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            turret.setRotation(cX, cY, cR, tX, tY);
            turret.updatePID();
        }
    }
}