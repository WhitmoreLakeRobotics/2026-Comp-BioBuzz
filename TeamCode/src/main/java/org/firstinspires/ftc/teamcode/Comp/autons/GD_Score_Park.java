package org.firstinspires.ftc.teamcode.Comp.autons;

import static com.pedropathing.ivy.Scheduler.schedule;
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

import org.firstinspires.ftc.teamcode.Comp.Hardware.Robot;

@Autonomous
        (name = "Gd_Score_Park", group = "")
public class GD_Score_Park extends OpMode {


    Robot robot = new Robot();
    private TelemetryManager telemetryM;



    public static final PoseFactory pf = PoseFactory.degrees();

/*  Replaced these with calls to AutonBase poses

    public static Pose startpose = pf.of(56, 8, 90);
    public static Pose scorePose = pf.of(56, 25, 90);
    public static Pose parkpose = pf.of(10, 90, 90);
*/
    public static Pose CPparkpath = pf.of(24, 39, 0);


    public Path pth1Score() {
        return Paths.line(AutonBase.GDstartPose, AutonBase.GDscorePose).constant(AutonBase.GDscorePose);
    }

    public Path pthPark() {
        return Paths.curve(AutonBase.GDscorePose, CPparkpath, AutonBase.GDparkPose).constant(AutonBase.GDparkPose);
    }

    protected Command cmdScorePark() {
        return sequential(
                robot.driveTrain.cmdAtFollow(pth1Score()),
                robot.cmdShootSequence(),
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
