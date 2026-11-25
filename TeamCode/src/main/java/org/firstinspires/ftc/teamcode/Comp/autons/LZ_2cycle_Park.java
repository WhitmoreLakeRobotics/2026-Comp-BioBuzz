package org.firstinspires.ftc.teamcode.Comp.autons;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;


;
import static org.firstinspires.ftc.teamcode.Comp.autons.AutonBase.GDscoreCenterPose;
import static org.firstinspires.ftc.teamcode.Comp.autons.AutonBase.GardenPose;
import static org.firstinspires.ftc.teamcode.Comp.autons.AutonBase.LZparkPose;
import static org.firstinspires.ftc.teamcode.Comp.autons.AutonBase.LZscoreCenterPose;
import static org.firstinspires.ftc.teamcode.Comp.autons.AutonBase.LZstartPose;
import static org.firstinspires.ftc.teamcode.Comp.autons.AutonBase.OpGDFlowerPose;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
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
        return Paths.line(LZstartPose, LZscoreCenterPose).linear(LZstartPose, LZscoreCenterPose);

    }

    public Path pthFlower() {
        return Paths.line(LZscoreCenterPose, OpGDFlowerPose).linear(LZscoreCenterPose, OpGDFlowerPose);
    }

    public Path pthScore2() {
        return Paths.line(OpGDFlowerPose, GDscoreCenterPose).linear(OpGDFlowerPose, GDscoreCenterPose);
    }

    public Path pthgarden() {
        return Paths.line(GDscoreCenterPose,GardenPose).linear(GDscoreCenterPose,GardenPose);
    }
    public  Path pthscore3(){
        return Paths.line(GardenPose, GDscoreCenterPose).linear(GardenPose, GDscoreCenterPose);
    }
    public Path pthPark() {
        return Paths.line(GDscoreCenterPose, LZparkPose).linear(GDscoreCenterPose, LZparkPose);

    }


    protected Command cmdLZcycle1() {
        return sequential(
                robot.driveTrain.cmdAtFollow(pthScore1()),
                robot.cmdShootSequence(),
                waitMs(4000),                              // keep firing; time how long your volley take
                robot.driveTrain.cmdAtFollow(pthFlower()),
                //flower extracter ?
                robot.driveTrain.cmdAtFollow(pthScore2()),
                robot.cmdShootSequence(),
                waitMs(4000),                              // keep firing; time how long your volley take
                robot.driveTrain.cmdAtFollow(pthgarden()),
                robot.cmdIntakeatSpeed(),
                robot.driveTrain.cmdAtFollow(pthscore3()),
                robot.cmdShootSequence(),
                waitMs(4000),                              // keep firing; time how long your volley take
                robot.driveTrain.cmdAtFollow(pthPark()),
                robot.cmdTripleThreatStop(),
                robot.driveTrain.cmdHoldPos(LZparkPose),
                robot.cmdStopIntakeFeeder()

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
