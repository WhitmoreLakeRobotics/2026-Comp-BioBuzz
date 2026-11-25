package org.firstinspires.ftc.teamcode.Comp.autons;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.Comp.Hardware.Robot;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.conditional;
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
import com.qualcomm.robotcore.util.ElapsedTime; // MJD

import org.firstinspires.ftc.teamcode.Comp.Hardware.ScorePoses;

@Autonomous(name = "GD_Waits_2ndTip", group = "") // MJD: was missing, so GD_Waits_2ndTip never showed up on the Driver Station
public class GD_Waits_2ndTip extends OpMode {

    Robot robot = new Robot();
    public static final PoseFactory pf = PoseFactory.degrees();
    private final ElapsedTime spinTimer = new ElapsedTime();
    private static final double HIVE_VERIFY_MAX_SEC = 2.0; // MJD: longest we wait to see the hive tip before going on anyway
    private static final double SPINUP_WAIT_MAX_SEC = 4.0;
    private static final double AUTO_LENGTH_SEC = 30.0; // MJD: length of the autonomous period
    private static final double PARK_TIME_LEFT_SEC = 3.0; // MJD: with this much time left we stop waiting and go park
    private final ElapsedTime autoTimer = new ElapsedTime(); // MJD: reset in start()
    private TelemetryManager telemetryM; // MJD
    private Command auto; // MJD

    /** MJD: seconds left in the autonomous period. */
    private double timeLeftSec() { // MJD
        return AUTO_LENGTH_SEC - autoTimer.seconds(); // MJD
    } // MJD

    public static final Pose backup = pf.of (11,24.6,230);

    public Path ScoreOne(){
        return Paths.line(AutonBase.GDstartPose,AutonBase.GDscorePose) .linear(AutonBase.GDstartPose,AutonBase.GDscorePose);
    }

    public static final Pose GDcp = pf.of(11.71, 50.27, 270); // MJD: control point that makes the garden drive an arc (same as GD_2tip_park)

    //public Path GdPickup(){
    //    return Paths.line(AutonBase.GDscorePose,AutonBase.GDpickup).linear(AutonBase.GDscorePose,AutonBase.GDpickup);
    //}   // MJD: old, a straight line with the same heading at both ends, so the robot strafed straight over

    public Path GdPickup(){ // MJD: curve through GDcp so the robot drives an arc
        return Paths.curve(AutonBase.GDscorePose,GDcp,AutonBase.GDpickup).linear(AutonBase.GDscorePose,AutonBase.GDpickup); // MJD
    } // MJD

    public Path Backup(){
        return Paths.line(AutonBase.GDpickup,backup).linear(AutonBase.GDpickup,backup);
    }

    //public Path Park(){
    //    return Paths.line(backup,AutonBase.GDparkPose).linear(backup,AutonBase.GDparkPose);
    //}   // MJD: old, parked from the backup pose. Now we park from the score pose.

    public Path ToScore(){ // MJD: backup pose to the score pose (LB)
        return Paths.line(backup,ScorePoses.ScorePoseLB()).linear(backup,ScorePoses.ScorePoseLB()); // MJD
    } // MJD

    public Path Park(){ // MJD: park from the score pose
        return Paths.line(ScorePoses.ScorePoseLB(),AutonBase.GDparkPose).linear(ScorePoses.ScorePoseLB(),AutonBase.GDparkPose); // MJD
    } // MJD

    protected Command OneCycle(){
        return sequential(
                robot.driveTrain.cmdAtFollow(ScoreOne()),
                instant(() -> spinTimer.reset()),
                waitUntil(() -> robot.launcher.isLauncherAtSpeed() || spinTimer.seconds() > SPINUP_WAIT_MAX_SEC), // MJD: the score path is short, let the launcher get up to speed
                instant(() -> robot.limelight.armHiveBaseline()), // MJD: remember where the hive is at the first launch
                robot.cmdLaunchVolley(AutonBase.LAUNCH_TIMER),
                robot.cmdCeaseFireSequence(),
                robot.driveTrain.cmdAtFollow(GdPickup()).raceWith(waitMs(AutonBase.PICKUP_MAX_MS)),
                robot.driveTrain.cmdAtFollow(Backup()),
                robot.driveTrain.cmdAtFollow(ToScore()), // MJD: drive to the score pose (LB); the hive check and launch happen from here
                instant(() -> spinTimer.reset()),
                //waitUntil(() -> robot.limelight.getHiveState() != Limelight.HiveState.UNKNOWN || spinTimer.seconds() > HIVE_VERIFY_MAX_SEC), // MJD: old, any reading ended the wait
                waitUntil(() -> robot.limelight.isHiveUpright() || spinTimer.seconds() > HIVE_VERIFY_MAX_SEC), // MJD: give the camera time to see the hive in its original spot
                conditional( // MJD
                        //() -> robot.limelight.isHiveTipped(),   // MJD: old, a hive that was NOT seen counted as "not tipped" and we launched right away
                        () -> !robot.limelight.isHiveUpright(), // MJD: tipped = the tag is not back in its original spot (moved OR not visible)
                        sequential( //naj waiting until either the hive is upright or the time is up
                                waitUntil(() -> robot.limelight.isHiveUpright() || timeLeftSec() <= PARK_TIME_LEFT_SEC), // MJD
                                conditional(
                                        () -> robot.limelight.isHiveUpright() && timeLeftSec() > PARK_TIME_LEFT_SEC, //naj: the hive is upright and we have enough time left
                                        robot.cmdLaunchVolley(AutonBase.LAUNCH_TIMER),
                                        instant(() -> { })
                                )
                        ),
                        robot.cmdLaunchVolley(AutonBase.LAUNCH_TIMER) // MJD: hive is upright, launch now
                ),
                robot.cmdCeaseFireSequence(),
                robot.driveTrain.cmdAtFollow(Park()),
                robot.cmdTripleThreatStop(),
                robot.cmdStopIntakeFeeder()
        );
    }

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
        autoTimer.reset();
        robot.launcher.cmdLauncherForward();
        robot.intake.cmdIntakeForward();
        auto = OneCycle();
        schedule(auto);
    }

    @Override
    public void loop() {
        robot.loop(); // MJD
        Scheduler.execute(); // MJD
        telemetryM.addData("pose", robot.driveTrain.getPose()); // MJD
        telemetryM.addData("hive state", robot.limelight.getHiveState()); // MJD
        telemetryM.addData("time left", timeLeftSec()); // MJD
        telemetryM.update(); // MJD
        telemetry.addData("auto running", auto.isScheduled()); // MJD
        telemetry.addData("hive state", robot.limelight.getHiveState()); // MJD
        telemetry.addData("time left (s)", "%.1f", timeLeftSec()); // MJD
        telemetry.update(); // MJD
    }

    @Override
    public void stop() {
        robot.stop(); // MJD
    }
}
