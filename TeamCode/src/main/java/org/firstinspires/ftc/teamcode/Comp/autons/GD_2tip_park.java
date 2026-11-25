package org.firstinspires.ftc.teamcode.Comp.autons;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.instant; // MJD
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil; // MJD
import static com.pedropathing.ivy.groups.Groups.sequential;

import static org.firstinspires.ftc.teamcode.Comp.autons.AutonBase.LZscorePose;
import static org.firstinspires.ftc.teamcode.Comp.autons.AutonBase.LZstartPose;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime; // MJD

import org.firstinspires.ftc.teamcode.Comp.Hardware.Flower_Ramp;
import org.firstinspires.ftc.teamcode.Comp.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Comp.Hardware.ScorePoses;

@Autonomous (name = "GD_2tip_park", group = "")
public class GD_2tip_park extends OpMode {


    Robot robot = new Robot();
    private TelemetryManager telemetryM;
    private final ElapsedTime spinTimer = new ElapsedTime(); // MJD
    private static final double HIVE_VERIFY_MAX_SEC = 2.0; // MJD: longest we wait to see the hive tip before going on anyway
    private static final double SPINUP_WAIT_MAX_SEC = 4.0; // MJD: longest we wait for the launcher before firing anyway



    public static final PoseFactory pf = PoseFactory.degrees();

    /*  Replaced these with calls to AutonBase poses

        public static Pose startpose = pf.of(56, 8, 90);
        public static Pose scorePose = pf.of(56, 25, 90);
        public static Pose parkpose = pf.of(10, 90, 90);
    */
    public static Pose CPparkpath = pf.of(24, 39, 0);
  //  public static Pose GDpickup = pf.of(10.6,7.7,180);
    public static Pose GDcp = pf.of (11.71, 50.27, 270);
    public static Pose Backup = pf.of (14, 25.3, 270);
    public static Pose park = pf.of (14.2, 115, 225);
    public static Pose flowerpickup = pf.of (46.7, 132, 90);
    public static Pose toscore = pf.of(32, 83,270);
    public static Pose flowerBack = pf .of(47, 108,90);
   // public static Pose



    public Path pth1Score() {
        return Paths.line(AutonBase.GDstartPose, AutonBase.GDscorePose).constant(AutonBase.GDscorePose);
    }

  /*  public Path pthPark() {
        return Paths.curve(AutonBase.GDscorePose, CPparkpath, AutonBase.GDparkPose).constant(AutonBase.GDparkPose);
    }

   */
    public Path Grdnpickup(){
        return Paths.curve(AutonBase.GDscorePose,GDcp, AutonBase.GDpickup). linear(AutonBase.GDscorePose, AutonBase.GDpickup);
    }
    public Path backup(){
        return Paths.line(AutonBase.GDpickup, Backup).linear(AutonBase.GDpickup, Backup);
    }
    public Path limecheck(){
        return Paths.line(Backup,ScorePoses.ScorePoseLB()).linear(Backup,ScorePoses.ScorePoseLB());
    }
    public Path limebackontrack(){
        return Paths.line(ScorePoses.ScorePoseLB(),toscore).linear(ScorePoses.ScorePoseLB(),toscore);
    }
    public Path toscore(){
        return Paths.line(Backup,toscore).linear(Backup,toscore);
    }
    public Path ToLaunch(){
        return Paths.line(toscore,LZscorePose).linear(toscore,LZscorePose);
    }
    public Path flowerback() {
        return Paths.line(LZscorePose, flowerBack).linear(LZscorePose, flowerBack);
    }

    public Path flower(){
        return Paths.line(flowerBack,flowerpickup).linear(flowerBack,flowerpickup);
    }
    public Path flowerback2(){
        return Paths.line(flowerpickup,flowerBack).linear(flowerpickup,flowerBack);
    }
    public Path Toscore2(){
        return Paths.line(flowerBack,ScorePoses.ScorePoseLT()).linear(flowerBack,ScorePoses.ScorePoseLT());
    }
    public Path prkPth(){
        return Paths.line(ScorePoses.ScorePoseLT(), AutonBase.GDparkPose).linear(ScorePoses.ScorePoseLT(), AutonBase.GDparkPose);
    }


    protected Command cmdScorePark() {
        // MJD: the score path is only ~2 in long, so the volley used to start right after PLAY, before the
        // launcher was up to speed. TripleThreat then timed out (1.5 s), aborted, and nothing fired.
        // Now we wait here (up to SPINUP_WAIT_MAX_SEC) for the launcher to reach speed first.
        //return sequential(
        //        robot.driveTrain.cmdAtFollow(pth1Score()),
        //        robot.cmdLaunchVolley(4000),  // tune ms to finish all 4 pollen
        //        robot.driveTrain.cmdAtFollow(pthPark())
        //);
        return sequential( // MJD
                robot.driveTrain.cmdAtFollow(pth1Score()), // MJD
                instant(() -> spinTimer.reset()), // MJD
                waitUntil(() -> robot.launcher.isLauncherAtSpeed() || spinTimer.seconds() > SPINUP_WAIT_MAX_SEC), // MJD
                instant(() -> robot.limelight.armHiveBaseline()), // MJD: remember where the hive is at the first launch
                robot.cmdLaunchVolley(AutonBase.LAUNCH_TIMER),  // tune ms to finish all 4 pollen
                robot.cmdCeaseFireSequence(),
               // robot.driveTrain.cmdAtFollow(Grdnpickup()),
                robot.driveTrain.cmdAtFollow(Grdnpickup()).raceWith(waitMs(AutonBase.PICKUP_MAX_MS)),
                robot.driveTrain.cmdAtFollow(backup()),
                //waitUntil(() -> robot.limelight.isHiveUpright() || spinTimer.seconds() > SPINUP_WAIT_MAX_SEC),
                //conditional(// MJD
                ////() -> robot.limelight.isHiveTipped(),   // MJD: old, a hive that was NOT seen counted as "not tipped" and we launched right away
                //() -> !robot.limelight.isHiveUpright(), // MJD: tipped = the tag is not back in its original spot (moved OR not visible)
                //instant(() -> { }),
                //sequential(
                //waitUntil(() -> robot.limelight.isHiveUpright()), // MJD
                //conditional(
                //() -> robot.limelight.isHiveUpright(),
                //sequential(
                //robot.driveTrain.cmdAtFollow(limecheck()),
                //robot.cmdLaunchVolley(AutonBase.LAUNCH_TIMER),
                //robot.driveTrain.cmdAtFollow(limebackontrack())
                //),
                //instant(() -> { })
                //),
                //instant(() -> { })
                //)
                //),
                //instant(() -> spinTimer.reset()), // MJD: verify the hive tipped (or give up after HIVE_VERIFY_MAX_SEC)
                ////   waitUntil(() -> robot.limelight.isHiveTipped() || spinTimer.seconds() > HIVE_VERIFY_MAX_SEC), // MJD
                //robot.driveTrain.cmdAtFollow(toscore()),
                // MJD: hive check. The old block above had these problems: spinTimer was not reset, so the 4 s wait was already over and the
                // check ended instantly; it had an unlimited waitUntil that could hang the auto; and after launching at LB it still ran toscore(),
                // which starts from Backup, so the robot would drive back toward Backup.
                // Now: wait up to HIVE_VERIFY_MAX_SEC to see the hive tag where it was at the first launch.
                //   upright (the hive did not tip) -> go to LB, launch what we picked up, then drive on to toscore
                //   tipped / tag not seen          -> skip the launch and drive straight to toscore
                //instant(() -> spinTimer.reset()), // MJD
                //waitUntil(() -> robot.limelight.isHiveUpright() || spinTimer.seconds() > HIVE_VERIFY_MAX_SEC), // MJD
                //conditional( // MJD
                //() -> robot.limelight.isHiveUpright(), // MJD
                //sequential( // MJD
                //robot.driveTrain.cmdAtFollow(limecheck()), // MJD: Backup -> LB
                //robot.cmdLaunchVolley(AutonBase.LAUNCH_TIMER), // MJD
                //robot.driveTrain.cmdAtFollow(limebackontrack()) // MJD: LB -> toscore (already there after this)
                //), // MJD
                //robot.driveTrain.cmdAtFollow(toscore()) // MJD: Backup -> toscore
                //), // MJD
                // MJD: version 3. The camera is on the launcher side, so the hive is only in view from the LB score pose
                // (the launcher faces it there), not from the backup pose. Drive to LB first, check the hive there,
                // launch if it is still upright, then go on to toscore from LB either way.
                robot.driveTrain.cmdAtFollow(limecheck()), // MJD: Backup -> LB
                instant(() -> spinTimer.reset()), // MJD
                waitUntil(() -> robot.limelight.isHiveUpright() || spinTimer.seconds() > HIVE_VERIFY_MAX_SEC), // MJD: wait up to 2 s to see the hive tag where it was
                conditional( // MJD
                        () -> robot.limelight.isHiveUpright(), // MJD: upright = the hive did not tip
                        robot.cmdLaunchVolley(AutonBase.LAUNCH_TIMER), // MJD: launch what we picked up, from LB
                        instant(() -> { }) // MJD: tipped or tag not seen, skip the launch
                ), // MJD
                robot.driveTrain.cmdAtFollow(limebackontrack()), // MJD: LB -> toscore, same for both outcomes
                robot.driveTrain.cmdAtFollow(ToLaunch()),
                robot.cmdLaunchVolley(AutonBase.LAUNCH_TIMER),
                robot.driveTrain.cmdAtFollow(flowerback()),
                robot.cmdDown(),
                robot.driveTrain.cmdAtFollow(flower()),
                robot.cmdIntakeatSpeed(),
                waitMs(AutonBase.Ramp_TIMER),
                robot.driveTrain.cmdAtFollow(flowerback2()),
                robot.cmdUp(),
                robot.driveTrain.cmdAtFollow(Toscore2()),
                robot.cmdLaunchVolley(AutonBase.LAUNCH_TIMER),
                robot.driveTrain.cmdAtFollow(prkPth()),
                robot.cmdTripleThreatStop(),
               // robot.driveTrain.cmdHoldPos(park),
                robot.cmdStopIntakeFeeder()
       ) ;

    }

    private Command auto;



    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        robot.hardwareMap = hardwareMap;
        robot.telemetry = telemetry;
        robot.init();
        Scheduler.reset();
        robot.driveTrain.cmdOpmodestartPose(AutonBase.GDstartPose);

    }


    @Override
    public void init_loop() {
        robot.init_loop();

    }


    @Override
    public void start() {
        robot.start();
        robot.launcher.cmdLauncherForward();
        robot.intake.cmdIntakeForward();
        auto = cmdScorePark();
        schedule(auto);


    }

    @Override
    public void loop() {
        robot.loop();
        Scheduler.execute();
        telemetryM.addData("pose", robot.driveTrain.getPose());
        telemetryM.addData("auto running", auto.isScheduled());
        telemetryM.addData("hive state", robot.limelight.getHiveState()); // MJD
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


