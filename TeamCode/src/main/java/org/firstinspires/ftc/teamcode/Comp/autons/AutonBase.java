package org.firstinspires.ftc.teamcode.Comp.autons;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;


public class AutonBase extends Object {


    public static Follower follower;
    // Pose factory that reads headings in DEGREES and converts them to radians for Pedro.
// Must be declared ABOVE the poses that use it.
    public static final PoseFactory Pf = PoseFactory.degrees();


    public static Pose GDstartPose = Pf.of(55, 8, 270);
    public static Pose GDscorePose = Pf.of(56, 12, 270);
    public static Pose GDparkPose = Pf.of(11, 90, 0);

    public static Pose LZstartPose = Pf.of(59, 136, 90);
    public static Pose LZscorePose = Pf.of(59, 125, 90);
    public static Pose LZparkPose = Pf.of(12, 120, 0);

    public static Pose HiveflowerPose = Pf.of(14, 57, 250);
    public static Pose LZFlowerPose = Pf.of(56, 128, 160);

    public static Pose GardenPose = Pf.of(9, 10, 270);
    public static Pose OpLZFlowerPose = Pf.of(130, 84, 73);
    public static Pose OpHiveFlowerPose = Pf.of(82, 112, -25);
    public static Pose AltLZscorePose = Pf.of(9, 10, 270);


    private static final Pose scorPoseA = Pf.of(30, 131, Math.toRadians(120)); //right side of the red hive relative to the driveteam
    private static final Pose scorePoseB = Pf.of(88.3,127, Math.toRadians(50)); // left side of the red hive on the right side relative to the driveteam
    private static final Pose scorePoseY = Pf.of(29, 15, Math.toRadians(230)); // left side of the red hive on the left side relative to the driveteam
    private static final Pose scorePoseX = Pf.of(86.4, 15, Math.toRadians(310)); // right side of the red hive on the left side relative to the driveteam

/* Changed class from Opmode to Object so these are not needed

    public void init(){


    }

    public void init_loop(){


    }

    public void start(){


    }

    public void loop(){


    }

    public void stop(){


    }

*/


}
