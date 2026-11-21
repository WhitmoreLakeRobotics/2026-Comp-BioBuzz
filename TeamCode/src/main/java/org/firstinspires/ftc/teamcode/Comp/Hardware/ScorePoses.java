package org.firstinspires.ftc.teamcode.Comp.Hardware;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

@Configurable
public class ScorePoses  {
    private static final PoseFactory pf = PoseFactory.degrees();

    public static double LT_X = 30.7,LT_Y = 129.8, LT_Heading = 120;
    public static double RT_X = 78.2, RT_Y = 134.5, RT_Heading = 63.2; // y was 124
    public static double LB_X = 27, LB_Y = 15, LB_Heading = 233; //y was 15, h was 238, 230

    // MJD: RB is now an exact mirror of LB about x = 58 (the same line LT/RT use): X = 116 - LB_X, Y = LB_Y, heading = 180 - LB_Heading.
    // MJD: previous RB values, in case you want them back:
    //   public static double RB_X = 83, RB_Y = 16, RB_Heading = 296;   // MJD: the RB right before the mirror
    //   public static double RB_X = 76, RB_Y = 15, RB_Heading = 290;   // MJD: the RB before that (x was 71, y was 15 before it)
    public static double RB_X = 86, RB_Y = 17, RB_Heading = 308; // MJD: mirror of LB (27, 15, 233)

    public static Pose ScorePoseLT(){
        return pf.of(LT_X, LT_Y,LT_Heading);
    } //Left Top
    public static Pose ScorePoseRT(){
        return pf.of(RT_X, RT_Y,RT_Heading);
    }  //Right Top
    public static Pose ScorePoseLB(){
        return pf.of(LB_X, LB_Y,LB_Heading);
    } //Left Bottom
    public static Pose ScorePoseRB(){
        return pf.of(RB_X, RB_Y,RB_Heading);
    } //Right Bottom

}
