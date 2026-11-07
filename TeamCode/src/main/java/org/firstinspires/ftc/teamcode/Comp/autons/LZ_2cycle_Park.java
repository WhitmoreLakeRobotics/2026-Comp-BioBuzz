package org.firstinspires.ftc.teamcode.Comp.autons;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;


import static org.firstinspires.ftc.teamcode.Comp.autons.AutonBase.GDparkPose;;
import static org.firstinspires.ftc.teamcode.Comp.autons.AutonBase.GDscorePose;
import static org.firstinspires.ftc.teamcode.Comp.autons.AutonBase.GardenPose;
import static org.firstinspires.ftc.teamcode.Comp.autons.AutonBase.LZparkPose;
import static org.firstinspires.ftc.teamcode.Comp.autons.AutonBase.LZscorePose;
import static org.firstinspires.ftc.teamcode.Comp.autons.AutonBase.LZstartPose;
import static org.firstinspires.ftc.teamcode.Comp.autons.AutonBase.OpGDFlowerPose;
import static org.firstinspires.ftc.teamcode.Comp.autons.GD1cycle_Park.scorePose;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Comp.Hardware.Robot;

// Starts next to on the flower on the LZ end goes a short distance up and go to score
// flower then parks on the LZ corner of loading zone
@Disabled
@Autonomous(name = "LZ_2cycle_Inpark", group = "")
public class LZ_2cycle_Park extends OpMode {

    Robot robot = new Robot();
    private TelemetryManager telemetryM;

    public static final PoseFactory pf = PoseFactory.degrees();
  /*  public static Pose startPose = pf.of(60, 132, 270);
    public static Pose scorePose = pf.of(60, 120, 275);
    public static Pose parkPose = pf.of(17, 112, 295);
    public static Pose flowerHivePose = pf.of(56, 128, 160);
*/
    //public static Pose Pickupgarden = pf.of(9, 10, 270);
    public Path pthScore1() {
        return Paths.line(LZstartPose, LZscorePose).linear(LZstartPose, LZscorePose);

    }

    public Path pthFlower() {
        return Paths.line(LZscorePose, OpGDFlowerPose).linear(LZscorePose, OpGDFlowerPose);
    }

    public Path pthScore2() {
        return Paths.line(OpGDFlowerPose, GDscorePose).linear(OpGDFlowerPose, GDscorePose);
    }

    public Path pthhive() {
        return Paths.line(GDscorePose,GardenPose).linear(GDscorePose,GardenPose);
    }
    public  Path pthscore3(){
        return Paths.line(GardenPose,GDscorePose).linear(GardenPose,GDscorePose);
    }
    public Path pthPark() {
        return Paths.line(GDscorePose, LZparkPose).linear(GDscorePose, LZparkPose);

    }


    protected Command cmdLZcycle1() {
        return sequential(
                robot.driveTrain.cmdAtFollow(pthScore1()),
                robot.driveTrain.cmdAtFollow(pthFlower()),
                robot.driveTrain.cmdAtFollow(pthScore2()),
                robot.driveTrain.cmdAtFollow(pthhive()),
                robot.driveTrain.cmdAtFollow(pthPark())
        );
    }

    private Command auto;

    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        robot.hardwareMap = hardwareMap;
        robot.telemetry = telemetry;
        robot.init();
        Scheduler.reset();
        robot.driveTrain.cmdOpmodestartPose(LZstartPose);

    }

    @Override
    public void init_loop() {
        robot.init_loop();

    }

    @Override
    public void start() {
        robot.start();
        auto = cmdLZcycle1();
        schedule(auto);

    }

    @Override
    public void loop() {
        robot.loop();
        Scheduler.execute();
        telemetryM.addData("pose", robot.driveTrain.getPose());
        telemetryM.addData("auto running", auto.isScheduled());
        telemetryM.update();   // sends to Panels only

        // Driver Station: only the few things the drive team needs
        telemetry.addData("auto running", auto.isScheduled());
        telemetry.update();

    }

    @Override
    public void stop() {
        robot.stop();

    }


}
