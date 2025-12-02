package org.firstinspires.ftc.teamcode;

public class Turret {
    /**
     * <p>Calculate the angle needed for the robot's turret to aim to a specific position on the field.<br />Note that <span style="font-style:italic">either degrees or radians can be used</span>, but you have to be consistent.<br /><br />&quot;I &lt;3 trigonometry!&quot;</p>
     * @param cX The current X position of Mr. Roboto
     * @param cY The current Y position of Mr. Roboto
     * @param cR The current rotation of Mr. Roboto
     * @param tX The target's X position
     * @param tY The target's Y position
     * @return The target angle for Mr. Roboto's turret
     */
    public static double calculateTurretAngle(double cX, double cY, double cR, double tX, double tY) {
        double a = tX - cX;
        double b = tY - cY;
        double c = Math.sqrt((a * a) + (b * b));
        double A = Math.asin(1 / c * a);
        // double B = Math.asin(1 / c * b);
        double target = cR - A;
        return target;
    }
}
