package org.firstinspires.ftc.teamcode.Comp.autons;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant; // MJD
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil; // MJD
import static com.pedropathing.ivy.groups.Groups.sequential;

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

import org.firstinspires.ftc.teamcode.Comp.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Comp.Hardware.ScorePoses;

@Autonomous(name = "GD_2cycle_park", group = "")
public class GD_2cycle_park extends OpMode {


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
    public static Pose toscore = pf.of(32, 83,270);
    public static Pose CPparkpath = pf.of(24, 39, 0);
    //  public static Pose GDpickup = pf.of(10.6,7.7,180);
    public static Pose GDcp = pf.of(11.71, 50.27, 270);
    public static Pose Backup = pf.of(14, 25.3, 270);
    public static Pose cpGDtoLTPOSE = pf.of(39, 40, 270); //naj control point to avoid parked partner
    public static Pose park = pf.of(14.2, 115, 225);
    public static Pose flowerpickup = pf.of(46.7, 128, 90);
    // public static Pose


    public Path pth1Score() {
        return Paths.line(AutonBase.GDstartPose, AutonBase.GDscoreCenterPose).constant(AutonBase.GDscoreCenterPose);
    }

    /*  public Path pthPark() {
          return Paths.curve(AutonBase.GDscorePose, CPparkpath, AutonBase.GDparkPose).constant(AutonBase.GDparkPose);
      }

     */
    public Path Grdnpickup() {
        return Paths.curve(AutonBase.GDscoreCenterPose, GDcp, AutonBase.GDpickupPose).linear(AutonBase.GDscoreCenterPose, AutonBase.GDpickupPose);
    }

    public Path backup() {
        return Paths.line(AutonBase.GDpickupPose, Backup).linear(AutonBase.GDpickupPose, Backup);
    }
  public Path toscore(){
        return Paths.line(Backup,toscore).linear(Backup,toscore);
  }
    //back to launch at first hive again
    public Path ToLaunchA() {
        return Paths.line(toscore, ScorePoses.ScorePoseLB()).linear(toscore, ScorePoses.ScorePoseLB());
    }

    // from 2nd launch to flower on the other side
    public Path ToFlower() {
        return Paths.line(ScorePoses.ScorePoseLB(), flowerpickup).linear(ScorePoses.ScorePoseLB(), flowerpickup);
    }

    public Path FlrTOLaunchLT() {
        return Paths.line(flowerpickup, ScorePoses.ScorePoseLT()).linear(flowerpickup, ScorePoses.ScorePoseLT());

    }

    public Path ToLaunchLT() {
        return Paths.curve(Backup, cpGDtoLTPOSE, ScorePoses.ScorePoseLT()).linear(Backup, ScorePoses.ScorePoseLT());
    }

    public Path prkPth() {
        return Paths.line(ScorePoses.ScorePoseLT(), AutonBase.GDparkPose).linear(ScorePoses.ScorePoseLT(), AutonBase.GDparkPose);
    }

    protected Command cmdRtnScoreHive1ToFlwrToHive2() {
        return sequential(
                robot.driveTrain.cmdAtFollow(ToLaunchA()), //naj From Gdn to Launch button A
                robot.cmdLaunchVolley(AutonBase.LAUNCH_TIMER),
                //naj need to put flower ramp down here might want to spin first here so it runs straight instead of strafing
                robot.driveTrain.cmdAtFollow(ToFlower()),
                robot.cmdLaunchVolley(AutonBase.LAUNCH_TIMER),
                robot.driveTrain.cmdAtFollow(prkPth())
        );

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
                instant(() -> spinTimer.reset()), // MJD: verify the hive tipped (or give up after HIVE_VERIFY_MAX_SEC)
                waitUntil(() -> robot.limelight.isHiveTipped() || spinTimer.seconds() > HIVE_VERIFY_MAX_SEC), // MJD

                // put a conditional branch here

         /*       conditional(
                        () -> robot.limelight.isHiveTipped(),            // your check: return true if tip up
                        sequential(                     // PATH ONE (tip up)
                                //insert retry command sequence here
                        ),
                        sequential(                     // PATH TWO (tip down)
                                //insert command to go to score pose x, check if hive up then launch or park
                        )
                ),
                //this would replace the rest of the actions?
*/
                robot.driveTrain.cmdAtFollow(ToLaunchLT()),
                robot.cmdLaunchVolley(AutonBase.LAUNCH_TIMER),
                robot.driveTrain.cmdAtFollow(prkPth()),
                robot.cmdTripleThreatStop(),
                // robot.driveTrain.cmdHoldPos(park),
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


