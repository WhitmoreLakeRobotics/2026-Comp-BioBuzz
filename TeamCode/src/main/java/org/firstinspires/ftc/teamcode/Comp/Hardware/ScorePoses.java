package org.firstinspires.ftc.teamcode.Comp.Hardware;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

@Configurable
public class ScorePoses  {
    private static final PoseFactory pf = PoseFactory.degrees();

    public static double LT_X = 31,LT_Y = 130, LT_Heading = 120;
    public static double RT_X = 85, RT_Y = 128, RT_Heading= 60; // y was 124
    public static double LB_X = 29, LB_Y = 17, LB_Heading = 238; //y was 15
    public static double RB_X = 76, RB_Y = 15, RB_Heading = 290; // x was 71

    public static Pose ScorePoseLT(){
        return pf.of(LT_X, LT_Y,LT_Heading);
    }
    public static Pose ScorePoseRT(){
        return pf.of(RT_X, RT_Y,RT_Heading);
    }
    public static Pose ScorePoseLB(){
        return pf.of(LB_X, LB_Y,LB_Heading);
    }
    public static Pose ScorePoseRB(){
        return pf.of(RB_X, RB_Y,RB_Heading);
    }

}
