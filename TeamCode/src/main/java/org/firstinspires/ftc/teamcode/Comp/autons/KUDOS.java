package org.firstinspires.ftc.teamcode.Comp.autons;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.Comp.Hardware.Robot;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.instant; // MJD
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil; // MJD
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

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

public class KUDOS extends OpMode {

    Robot robot = new Robot();
    public static final PoseFactory pf = PoseFactory.degrees();
    private final ElapsedTime spinTimer = new ElapsedTime();
    private static final double HIVE_VERIFY_MAX_SEC = 2.0; // MJD: longest we wait to see the hive tip before going on anyway
    private static final double SPINUP_WAIT_MAX_SEC = 4.0;

    public static final Pose backup = pf.of (11,24.6,230);

    public Path ScoreOne(){
        return Paths.line(AutonBase.GDstartPose,AutonBase.GDscorePose) .linear(AutonBase.GDstartPose,AutonBase.GDscorePose);
    }

    public Path GdPickup(){
        return Paths.line(AutonBase.GDscorePose,AutonBase.GDpickup).linear(AutonBase.GDscorePose,AutonBase.GDpickup);
    }

    public Path Backup(){
        return Paths.line(AutonBase.GDpickup,backup).linear(AutonBase.GDpickup,backup);
    }

    public Path Park(){
        return Paths.line(backup,AutonBase.GDparkPose).linear(backup,AutonBase.GDparkPose);
    } // may not need

    protected Command OneCycle(){
        return sequential(
                robot.driveTrain.cmdAtFollow(ScoreOne()),
                robot.cmdLaunchVolley(AutonBase.LAUNCH_TIMER),
                robot.cmdCeaseFireSequence(),
                robot.driveTrain.cmdAtFollow(GdPickup()),
                robot.driveTrain.cmdAtFollow(Backup()),
                waitUntil(() -> robot.limelight.isHiveTipped() || spinTimer.seconds() > HIVE_VERIFY_MAX_SEC), // MJD
                robot.cmdLaunchVolley(AutonBase.LAUNCH_TIMER),
                robot.driveTrain.cmdAtFollow(Park()),
                robot.cmdTripleThreatStop()

        );
    }

    @Override
    public void init() {

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {

    }

    @Override
    public void stop() {

    }
}
