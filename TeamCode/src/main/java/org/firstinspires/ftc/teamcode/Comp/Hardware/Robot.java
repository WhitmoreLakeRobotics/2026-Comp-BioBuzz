package org.firstinspires.ftc.teamcode.Comp.Hardware;


import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;

/**
 * Holds all of Team 8492's hardware and passes each OpMode step
 * (init, init_loop, start, loop, stop) down to every part.
 * <p>
 * CHANGED: removed the duplicate "telemetry" and "hardwareMap" fields.
 * Robot extends BaseHardware, which already has both, so declaring them
 * again here created a second, confusing copy. Telop's
 * "robot.hardwareMap = hardwareMap;" still works exactly the same.
 */
public class Robot extends BaseHardware {

    public DriveTrain driveTrain = new DriveTrain();
    public Intake intake = new Intake();
    public Launcher launcher = new Launcher();
    // CHANGED: the Comp bot has no TransitionR servo.
    //public TransitionR transitionR = new TransitionR();
    public PollenBlocker pollenBlocker = new PollenBlocker();
    public Feeder feeder = new Feeder();
    public Flower_Ramp Ramp = new Flower_Ramp();
    // CHANGED: runs the launch sequence. Declared AFTER the parts it uses.
    public TripleThreat tripleThreat = new TripleThreat(launcher, feeder, pollenBlocker, intake);
    public Lighting lighting = new Lighting();
    public Limelight limelight = new Limelight();
    public Flower_Ramp flower_ramp = new Flower_Ramp();


    /**
     * Called once when the INIT button is pressed.
     */
    @Override
    public void init() {
//checking to make sure running the right code for this robot markerDevice is a device that is unique in each robot
        // CHANGED: "TransS" is the Scrimmage servo and doesn't exist on the Comp bot.
        // TODO: put a device name that only the Comp bot has here.
        //CommonLogic.WhichRobotCheck(this.hardwareMap,"TransS","Competition");
        // OPTIONAL: once CommonLogic has the NotRobotCheck method, this fails if the Scrimmage
        // servo is present (wrong robot). Uncomment it then.
        //CommonLogic.NotRobotCheck(this.hardwareMap, "TransS", "Competition");
        // drivetrain
        driveTrain.hardwareMap = this.hardwareMap;
        driveTrain.telemetry = this.telemetry;
        driveTrain.init();

        // Intake
        intake.hardwareMap = this.hardwareMap;
        intake.telemetry = this.telemetry;
        intake.init();

        //Launcher
        launcher.hardwareMap = this.hardwareMap;
        launcher.telemetry = this.telemetry;
        launcher.init();

        //Transition Roller
        //transitionR.hardwareMap = this.hardwareMap;
        //transitionR.telemetry = this.telemetry;
        //transitionR.init();

        //launcher blocker
        pollenBlocker.hardwareMap = this.hardwareMap;
        pollenBlocker.telemetry = this.telemetry;
        pollenBlocker.init();

        //Feeder
        feeder.hardwareMap = this.hardwareMap;
        feeder.telemetry = this.telemetry;
        feeder.init();

        //Ramp
        Ramp.hardwareMap = this.hardwareMap;
        Ramp.telemetry = this.telemetry;
        Ramp.init();

        //TripleThreat (launch sequence)
        tripleThreat.hardwareMap = this.hardwareMap;
        tripleThreat.telemetry = this.telemetry;
        tripleThreat.init();

        //Lighting
        lighting.hardwareMap = this.hardwareMap;
        lighting.telemetry = this.telemetry;
        lighting.init();

        //Limelight
        limelight.hardwareMap = this.hardwareMap;
        limelight.telemetry = this.telemetry;
        limelight.init();

        //Flower_ramp
        flower_ramp.hardwareMap = this.hardwareMap;
        flower_ramp.telemetry = this.telemetry;
        flower_ramp.init();

    }

    @Override
    public void init_loop() {
        driveTrain.init_loop();
        intake.init_loop();
        launcher.init_loop();
        Ramp.init_loop();
        //transitionR.init_loop();
        pollenBlocker.init_loop();
        feeder.init_loop();
        tripleThreat.init_loop();
        lighting.init_loop();
        limelight.init_loop();
        flower_ramp.init_loop();
    }

    @Override
    public void start() {
        driveTrain.start();
        intake.start();
        launcher.start();
        Ramp.start();
        //transitionR.start();
        pollenBlocker.start();
        feeder.start();
        tripleThreat.start();
        lighting.start();
        limelight.start();
        flower_ramp.start();
    }

    @Override
    public void loop() {
        driveTrain.loop();
        intake.loop();
        Ramp.loop();
        // CHANGED: TripleThreat runs before the parts it commands, so its commands take effect this loop.
        tripleThreat.loop();
        launcher.loop();
        //transitionR.loop();
        pollenBlocker.loop();
        feeder.loop();
        lighting.loop();
        limelight.setRobotPose(driveTrain.getPose()); // MJD: so the hive tag can be placed on the field
        limelight.loop();
        flower_ramp.loop();
    }

    /**
     * Called once when STOP is pressed.
     * CHANGED: added "public" (was just "void stop()").
     * CHANGED: mechanisms stop first and driveTrain last (it saves the pose), so a failure in
     * driveTrain.stop() can't leave the launcher or feeder running.
     */
    @Override
    public void stop() {
        tripleThreat.stop();
        Ramp.stop();
        launcher.stop();
        feeder.stop();
        intake.stop();
        //transitionR.stop();
        pollenBlocker.stop();
        driveTrain.stop();
        lighting.stop();
        limelight.stop();
        flower_ramp.stop();
    }

    // ---------------- IVY COMMANDS (for Pedro auto paths) ----------------

    /**
     * Start the launch sequence and finish once TripleThreat is FIRING (or the shot aborted).
     * It keeps firing until cmdCeaseFireSequence() runs.
     * Auto must call robot.loop() every cycle, since that is what advances the states.
     */
    public Command cmdShootSequence() {
        return sequential(
                instant(() -> tripleThreat.cmdShoot()),
                waitUntil(() -> tripleThreat.isFiring() || tripleThreat.isIdle())
        ).requiring(this);
    }

    /** End the volley: stop the feeder and close the blocker (the launcher keeps spinning). */
    public Command cmdCeaseFireSequence() {
        return instant(() -> tripleThreat.cmdCeaseFire()).requiring(this);
    }
    public Command cmdIntakeatSpeed() {
        return sequential(
                instant(() -> intake.cmdIntakeForward()),
               waitMs(4000),instant(() -> intake.cmdIntakeStop()));
    }

    /** Spin up -> feed -> unblock, fire for ms, then feeder off and blocker closed.
     *  Finishes only after the volley. Launcher and intake are left running. */
    public Command cmdLaunchVolley(double ms) {
        return sequential(
                instant(() -> tripleThreat.cmdShoot()),
                instant(() -> tripleThreat.cmdShoot()), //need a double tap
                waitUntil(() -> tripleThreat.isFiring() || tripleThreat.isIdle()),
                waitMs(ms),                                   // timer starts once FIRING
                instant(() -> tripleThreat.cmdCeaseFire())
        );
    }

    // spinup launcher
     public Command cmdLaunchSpinup() {
         return sequential(
                 instant(() -> launcher.cmdLauncherForward()));
     }

    //stops intake and launcher
    public Command cmdTripleThreatStop(){
        return sequential(
              // instant(() -> launcher.cmdLauncherStop()),
               instant(() -> feeder.cmdFeederStop()),
               instant(() -> pollenBlocker.cmdBlock())
              // instant(() -> intake.cmdIntakeStop())
        );
    }
    public Command cmdStopIntakeFeeder(){
        return sequential(
                instant(() -> feeder.cmdFeederStop()),
                instant(() -> intake.cmdIntakeStop())
        );
    }



}