package org.firstinspires.ftc.teamcode;

public final class AngleUtil {
    private AngleUtil() {}

    /** Normalize an angle to [-PI, PI]. */
    public static double normDelta(double angleDelta) {
        return Math.atan2(Math.sin(angleDelta), Math.cos(angleDelta));
    }

    /** Normalize absolute angle to [0, 2PI). */
    public static double norm(double angle) {
        double a = angle % (2*Math.PI);
        if (a < 0) a += 2*Math.PI;
        return a;
    }
}
