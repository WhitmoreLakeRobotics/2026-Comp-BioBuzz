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

@Autonomous
        (name = "LZ_Score_Park", group = "")
public class LZ_Score_Park extends OpMode {


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
    public static Pose CPparkpath = pf.of(24, 129, 270);


    public Path pth1Forward() {
        return Paths.line(AutonBase.LZstartPose, AutonBase.LZscorePose).constant(AutonBase.LZscorePose);
    }
    public Path pth1Score() {
        return Paths.line(AutonBase.LZscorePose, ScorePoses.ScorePoseLT()).constant(ScorePoses.ScorePoseLT());
    }


    public Path pthPark() {
        return Paths.line(ScorePoses.ScorePoseLT(),AutonBase.LZparkPose).constant(AutonBase.LZparkPose);
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
                robot.driveTrain.cmdAtFollow(pth1Forward()),
                robot.driveTrain.cmdAtFollow(pth1Score()),
                robot.cmdLaunchSpinup(),
                waitMs(10000),
                instant(() -> spinTimer.reset()), // MJD
                waitUntil(() -> robot.launcher.isLauncherAtSpeed() || spinTimer.seconds() > SPINUP_WAIT_MAX_SEC), // MJD
                robot.cmdLaunchVolley(4000),  // tune ms to finish all 4 pollen
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
        robot.driveTrain.cmdOpmodestartPose(AutonBase.LZstartPose);

    }


    @Override
    public void init_loop() {
        robot.init_loop();

    }


    @Override
    public void start() {
        robot.start();
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
