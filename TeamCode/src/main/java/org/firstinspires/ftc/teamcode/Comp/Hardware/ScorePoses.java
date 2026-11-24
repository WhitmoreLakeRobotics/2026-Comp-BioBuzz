package org.firstinspires.ftc.teamcode.Comp.Hardware;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

@Configurable
public class ScorePoses  {
    private static final PoseFactory pf = PoseFactory.degrees();

    public static double LT_X = 30,LT_Y = 129, LT_Heading = 130;
    public static double RT_X = 86.5, RT_Y = 131, RT_Heading = 58; // y was 124
    public static double LB_X = 29.5, LB_Y = 19, LB_Heading = 231; //y was 15, h was 238, 230

    // MJD: RB is now an exact mirror of LB about x = 58 (the same line LT/RT use): X = 116 - LB_X, Y = LB_Y, heading = 180 - LB_Heading.
    // MJD: previous RB values, in case you want them back:
    //   public static double RB_X = 83, RB_Y = 16, RB_Heading = 296;   // MJD: the RB right before the mirror
    //   public static double RB_X = 76, RB_Y = 15, RB_Heading = 290;   // MJD: the RB before that (x was 71, y was 15 before it)
    public static double RB_X = 87, RB_Y = 19, RB_Heading = 303; // MJD: mirror of LB (27, 15, 233)

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
