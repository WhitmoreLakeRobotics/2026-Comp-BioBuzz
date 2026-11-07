package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;

/**
 * Pedro Pathing doesn't persist the robot's pose between OpModes on its own.
 * Autos save its ending pose here in stop(); TeleOp reads it in init()
 * so TeleOp can start from where autonomous left off.
 */
public class OpModeStorage {
    public static Pose EndPose = new Pose(8, 9, 0);  // nj changed default start to center of robot
    public static Pose autonomousstartPose = new Pose(0, 0, 0);
}
