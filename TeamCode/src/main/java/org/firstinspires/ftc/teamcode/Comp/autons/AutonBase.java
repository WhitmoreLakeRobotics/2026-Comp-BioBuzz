package org.firstinspires.ftc.teamcode.Comp.autons;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;


public class AutonBase {


    public static Follower follower;
    // Pose factory that reads headings in DEGREES and converts them to radians for Pedro.
// Must be declared ABOVE the poses that use it.
    public static final PoseFactory Pf = PoseFactory.degrees();

// as of 11/5/2026 the center of the robot is 8.25 inches on each side.
    // 7.5 from the launcher edge and 10.25 from the front of the intake.
    // PP actually sets to 0,0 off the field. the far wall is not 144, but 141.5 so don't measure from far wall
    public static Pose GDstartPose = Pf.of(55.25, 12.25, 270);
    public static Pose GDscorePose = Pf.of(55, 14, 270);
    public static Pose GDparkPose = Pf.of(13, 92, 0);

    public static Pose LZstartPose = Pf.of(59, 133.25, 90);
    public static Pose LZscorePose = Pf.of(59, 130, 90);
    public static Pose LZparkPose = Pf.of(13, 120, 0);

    public static Pose HiveflowerPose = Pf.of(35, 135, 250);
    public static Pose GDFlowerPose = Pf.of(10, 35, 160);
//op=opponents
    public static Pose GardenPose = Pf.of(9, 10, 270);
    public static Pose OpGDFlowerPose = Pf.of(130, 105, 73);
    public static Pose OpHiveFlowerPose = Pf.of(106, 13, 270);
    //public static Pose AltLZscorePose = Pf.of(9, 10, 270);

    public static final Pose scorePoseA = Pf.of(30, 131, 120); //right side of the red hive relative to the driveteam
    public static final Pose scorePoseB = Pf.of(88.3,127, 50); // left side of the red hive on the right side relative to the driveteam
    public static final Pose scorePoseY = Pf.of(29, 15, 230); // left side of the red hive on the left side relative to the driveteam
    public static final Pose scorePoseX = Pf.of(86.4, 15, 310); // right side of the red hive on the left side relative to the driveteam


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
