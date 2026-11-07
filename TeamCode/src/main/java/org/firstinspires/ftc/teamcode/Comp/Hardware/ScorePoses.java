package org.firstinspires.ftc.teamcode.Comp.Hardware;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

@Configurable
public class ScorePoses  {
    private static final PoseFactory pf = PoseFactory.degrees();

    public static double A_X = 30,A_Y = 31, A_Heading = 120;
    public static double B_X = 88.3, B_Y = 127, B_Heading= 50;
    public static double Y_X = 29, Y_Y = 15, Y_Heading = 230;
    public static double X_X = 86.4, X_Y = 15, X_Heading = 310;

    public static Pose ScorePoseA(){
        return pf.of(A_X, A_Y,A_Heading);
    }
    public static Pose ScorePoseB(){
        return pf.of(B_X, B_Y,B_Heading);
    }
    public static Pose ScorePoseY(){
        return pf.of(Y_X, Y_Y,Y_Heading);
    }
    public static Pose ScorePoseX(){
        return pf.of(X_X, X_Y,X_Heading);
    }

}
