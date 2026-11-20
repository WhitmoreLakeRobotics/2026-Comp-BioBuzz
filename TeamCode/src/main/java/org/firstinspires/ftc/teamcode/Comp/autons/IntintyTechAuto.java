package org.firstinspires.ftc.teamcode.Comp.autons;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.api.PoseFactory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Comp.Hardware.Robot;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Comp.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Comp.Hardware.ScorePoses;

@Disabled
public class IntintyTechAuto extends OpMode {

    Robot robot = new Robot();
    private TelemetryManager telemetryM;

    public static final PoseFactory pf = PoseFactory.degrees();

    public static final Pose LzStartPose = pf.of(61,133,90);
    public static final Pose lzPark = pf.of(14, 93,94);

    public Path ScorePath(){
        return Paths.line(LzStartPose,lzPark).linear(LzStartPose,lzPark);
    }

    protected Command cmdPth2Scr(){

        return sequential(
            robot.cmdLaunchVolley(4000),
            robot.driveTrain.cmdAtFollow(ScorePath()),
            robot.cmdTripleThreatStop()
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
        robot.driveTrain.cmdOpmodestartPose(LzStartPose);

    }

    @Override
    public void init_loop() {
        robot.init_loop();

    }

    @Override
    public void start() {
        robot.start();
        // robot.intake.cmdIntakeForward();
        auto = cmdPth2Scr() ;
        schedule(auto);

    }

    @Override
    public void loop() {
        robot.start();
    }

    @Override
    public void stop() {
        robot.stop();

    }
}
