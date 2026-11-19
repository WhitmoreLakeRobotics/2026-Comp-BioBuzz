package org.firstinspires.ftc.teamcode.Comp.autons;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
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
public class RoboWolvesauto extends OpMode {

    Robot robot = new Robot();

    public static final PoseFactory pf = PoseFactory.degrees();

    public static final Pose Scorepose = pf.of (61, 133, 90);
    public static final Pose Park = pf.of (11.4, 116, 90);


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
