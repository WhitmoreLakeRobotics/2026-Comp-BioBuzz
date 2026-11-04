package org.firstinspires.ftc.teamcode.Comp.autons;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


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
   
    public static Pose LZflowerPose = Pf.of(14, 57, 250);
    public static Pose LZendFlowerPose = Pf.of(56, 128, 160);
    public static Pose OpLZFlowerp3 = Pf.of(130, 84, 73);
    public static Pose OpLZendFlowerp4 = Pf.of(82, 112, -25);


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
