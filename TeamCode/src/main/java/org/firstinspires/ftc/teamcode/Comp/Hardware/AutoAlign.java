package org.firstinspires.ftc.teamcode.Comp.Hardware;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;

/**
 * MJD: auto align controller. It owns no hardware. Given the robot pose and the field position of a
 * target (an AprilTag from Limelight), it returns a turn power that points the LAUNCHER (the back of
 * the robot) at the target. It uses the Pinpoint heading, so the camera only has to have seen the tag once.
 * Turn power: + = turn left (Pedro convention).
 */
@Configurable
public class AutoAlign {

    // ---------------- TUNABLE CONSTANTS ----------------
    public static double ALIGN_KP = 0.02;          // turn power per degree of error
    public static double ALIGN_MIN_POWER = 0.08;   // enough to overcome friction
    public static double ALIGN_MAX_POWER = 0.6;
    public static double ALIGN_TOL_DEG = 1.5;      // aligned when closer than this
    public static double ALIGN_AIM_OFFSET_DEG = 0; // trim if the launcher is not exactly on the back axis

    // ---------------- STATE (for telemetry and auton "wait until aligned") ----------------
    private double lastErrDeg = 0;
    private boolean aligned = false;

    /** Turn power (+ = left) that points the back of the robot at tagField {x, y}. 0 when aligned or no target. */
    public double getTurnPower(Pose robot, double[] tagField) {
        if (robot == null || tagField == null) {
            aligned = false;
            return 0;
        }
        double desired = Math.atan2(tagField[1] - robot.y(), tagField[0] - robot.x())
                + Math.PI + Math.toRadians(ALIGN_AIM_OFFSET_DEG);                       // + 180 = launcher side
        double err = Math.atan2(Math.sin(desired - robot.heading()), Math.cos(desired - robot.heading())); // wrap to +-180
        lastErrDeg = Math.toDegrees(err);
        aligned = Math.abs(lastErrDeg) <= ALIGN_TOL_DEG;
        if (aligned) {
            return 0;
        }
        double power = ALIGN_KP * lastErrDeg;
        return Math.copySign(Math.max(ALIGN_MIN_POWER, Math.min(ALIGN_MAX_POWER, Math.abs(power))), power);
    }

    public boolean isAligned() { return aligned; }

    public double getErrorDeg() { return lastErrDeg; }
}
