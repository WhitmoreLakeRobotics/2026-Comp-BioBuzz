package org.firstinspires.ftc.teamcode.Comp.autons;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant; // MJD
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

@Autonomous (name = "GD_2tip_park", group = "")
public class GD_2tip_park extends OpMode {


    Robot robot = new Robot();
    private TelemetryManager telemetryM;
    private final ElapsedTime spinTimer = new ElapsedTime(); // MJD
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
    public Path ToLaunch(){
        return Paths.line(Backup, ScorePoses.ScorePoseLT()).linear(Backup, ScorePoses.ScorePoseLT());
    }
    public Path prkPth(){
        return Paths.line(ScorePoses.ScorePoseLT(), park).linear(ScorePoses.ScorePoseLT(), park);
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
                robot.cmdLaunchVolley(4000),  // tune ms to finish all 4 pollen
                robot.cmdCeaseFireSequence(),
                robot.driveTrain.cmdAtFollow(Grdnpickup()),
                robot.driveTrain.cmdAtFollow(backup()),
                robot.driveTrain.cmdAtFollow(ToLaunch()),
                robot.cmdLaunchVolley(4000),
                robot.driveTrain.cmdAtFollow(prkPth()),
                robot.cmdTripleThreatStop(),
                robot.driveTrain.cmdHoldPos(park)
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


