package org.firstinspires.ftc.teamcode.Comp.autons;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;

import static org.firstinspires.ftc.teamcode.Comp.autons.AutonBase.LZparkPose;
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

import org.firstinspires.ftc.teamcode.Comp.Hardware.Robot;

//starts in corner and goes to upper side park area red
@Autonomous(name = "LZPark", group = "")
public class LZPark extends OpMode {
    Robot robot = new Robot();
    private TelemetryManager telemetryM;



        private final PoseFactory Pf = PoseFactory.degrees();



        private final Pose point1Control1 = Pf.of(45, 97, 0);

    public Path pthpark1() {
        return Paths.curve(LZstartPose,point1Control1, LZparkPose).linear(LZstartPose, LZparkPose);

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
    protected Command cmdLZpark() {
        return sequential(
                robot.driveTrain.cmdAtFollow(pthpark1())


        );
    }


    @Override
    public void init_loop() {
        robot.init_loop();

    }


    @Override
    public void start() {
        robot.start();
        auto = cmdLZpark();
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
