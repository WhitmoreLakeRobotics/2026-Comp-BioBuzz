package org.firstinspires.ftc.teamcode.Comp;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime; // MJD

import org.firstinspires.ftc.teamcode.Common.CommonLogic;
import org.firstinspires.ftc.teamcode.Comp.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Comp.Hardware.Flower_Ramp; // MJD
import org.firstinspires.ftc.teamcode.Comp.Hardware.ScorePoses;
import org.firstinspires.ftc.teamcode.OpModeStorage;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;


// CHANGED: renamed so it doesn't clash with the Scrimmage TeleOp (two OpModes can't share a name).
@TeleOp(name = "TeleOp")
public class Telop extends OpMode {
private TelemetryManager telemetryM;
    private static final String TAGTeleop = "8492-Teleop";
    Robot robot = new Robot();
    private boolean debug = true;  // if debug, show telemetry with values you're interested in

    private boolean gp1_prev_a = false;
    private boolean gp1_prev_b = false;
    private boolean gp1_prev_x = false;
    private boolean gp1_prev_y = false;
    private boolean gp1_prev_right_bumper = false;
    private boolean gp1_prev_left_bumper = false;
    private boolean gp1_prev_dpad_up = false;
    private boolean gp1_prev_dpad_down = false;
    private boolean gp1_prev_dpad_left = false;
    private boolean gp1_prev_dpad_right = false;
    private boolean gp1_prev_back = false;
    private boolean gp1_prev_start = false;
    private boolean gp1_prev_right_trigger = false;
    // CHANGED: the shoot trigger moved to gamepad 2, so this flag is no longer used.
    //private boolean gp1_prev_right_trigger = false;

    private boolean gp2_prev_a = false;
    private boolean gp2_prev_b = false;
    private boolean gp2_prev_x = false;
    private boolean gp2_prev_y = false;
    private boolean gp2_prev_right_bumper = false;
    private boolean gp2_prev_left_bumper = false;
    private boolean gp2_prev_dpad_up = false;
    private boolean gp2_prev_dpad_down = false;
    private boolean gp2_prev_dpad_left = false;
    private boolean gp2_prev_dpad_right = false;
    private boolean gp2_prev_back = false;
    private boolean gp2_prev_start = false;
    // CHANGED: used to detect the moment the manual blocker trigger is released.
    private boolean gp2_prev_left_trigger = false;
    // CHANGED: used to detect the moment the shoot trigger is released (now on gamepad 2).
    private boolean gp2_prev_right_trigger = false;
    // CHANGED: replaced by the TripleThreat state machine (robot.tripleThreat), so this latch is no longer needed.
    //private boolean volleyActive = false;
    private double LeftMotorPower = 0;
    private double RightMotorPower = 0;
    private int tHeading = 0;
    private boolean bAutoTurn = false;

    // private boolean shootAborted = false;
    private boolean bAutoDrive = false;
    private boolean intakeReversePrev = false; // MJD: reverse button state last loop
    private boolean intakeWasForward = false; // MJD: was the intake going forward when reverse was pressed
    private final ElapsedTime autoDriveTimer = new ElapsedTime(); // MJD: time since the last A/B/X/Y drive-to-pose press
    private static final double AUTO_DRIVE_BREAKOUT_DELAY_SEC = 1.0; // MJD: sticks are ignored this long after a drive-to-pose press
    private boolean poseDriveCalled = false; // MJD: a drive-to-pose button was pressed and we have not left that spot

    // CHANGED: the drive speed the sticks use. dpad left/up/down set it (it stays until changed).
    private double driveSpeed = robot.driveTrain.NORMALSPEED;
    private Pose lastTarget = null;


    // CHANGED: Pedro headings are in RADIANS.
    private Pose op_pose = new Pose(90, 0, Math.toRadians(80));

    private Command DriveToCommand = null;
    public static final PoseFactory Pf = PoseFactory.degrees();
    private static final double InLaunchPosition = 3.5;

    private boolean InLaunchPosition(){
        Pose p = robot.driveTrain.getPose();
        for(Pose target: new Pose[]{ScorePoses.ScorePoseLB(), ScorePoses.ScorePoseRB(), ScorePoses.ScorePoseLT(), ScorePoses.ScorePoseRT()}){
            if(Math.hypot(p.x() - target.x(), p.y() - target.y()) <= InLaunchPosition){
                return true;
            }
        }
        return false;

    }

    private void goHold(Pose target, boolean SpinLauncher ){
        if (DriveToCommand != null && DriveToCommand.isScheduled()) {
            DriveToCommand.cancel();
        }
            DriveToCommand = robot.driveTrain.cmdHoldPos(target);
            lastTarget = target;
            poseDriveCalled = true; // MJD
            robot.launcher.setPoseSpeedMode(true); // MJD: go to the pose launch speed the moment the button is pressed
            autoDriveTimer.reset(); // MJD: start the no-breakout window
            Scheduler.schedule(DriveToCommand);
            if (SpinLauncher) {
                robot.launcher.cmdLauncherForward();
            }

    }

    @Override
    public void init() {
        Scheduler.reset();
        robot.hardwareMap = hardwareMap;
        robot.telemetry = telemetry;
        robot.init();

        // CHANGED: autonomousEndPose is null if TeleOp runs without Auto first, so use a fallback.
        Pose startPose = OpModeStorage.EndPose;
       // telemetryM.addData("startPose in tele init", startPose);
        if (startPose == null) {
            startPose = new Pose(8, 8, 8);   // TODO: set a real default start pose
        }
        robot.driveTrain.cmdOpmodestartPose(startPose);
    }

    @Override
    public void init_loop() {
        robot.init_loop();
        PanelsTelemetry.INSTANCE.getTelemetry().update();
        telemetry.update();
    }

    @Override
    public void start() {
        robot.start();
        gp1_prev_a = gamepad1.a;
        gp1_prev_b = gamepad1.b;
        gp1_prev_x = gamepad1.x;
        gp1_prev_y = gamepad1.y;

    }

    @Override
    public void loop() {

       /* if (bAutoTurn) {
            // auto-turn code goes here later (use FASTSPEED / MORNINGSPEED,
            // DTrain_FASTSPEED and DTrain_SLOWSPEED don't exist).
        } else {
            robot.driveTrain.cmdTeleOp(
                    CommonLogic.joyStickMath(gamepad1.left_stick_y * -1),
                    CommonLogic.joyStickMath(gamepad1.left_stick_x * -1),
                    CommonLogic.joyStickMath(gamepad1.right_stick_x),
                    driveSpeed);   // CHANGED: was robot.driveTrain.NORMALSPEED
        }

        */




        if (CommonLogic.oneShot(gamepad1.a,gp1_prev_a)){goHold(ScorePoses.ScorePoseLB(),true); bAutoDrive = true;}
        if (CommonLogic.oneShot(gamepad1.b,gp1_prev_b)){goHold(ScorePoses.ScorePoseRB(),true); bAutoDrive = true;}
        if (CommonLogic.oneShot(gamepad1.y,gp1_prev_y)){goHold(ScorePoses.ScorePoseRT(),true); bAutoDrive = true;}
        if (CommonLogic.oneShot(gamepad1.x,gp1_prev_x)){goHold(ScorePoses.ScorePoseLT(),true); bAutoDrive = true;}

        boolean alignHeld = gamepad1.left_trigger_pressed; // MJD: auto align to the closest positive AprilTag
        if (alignHeld && bAutoDrive) { // MJD: the trigger takes over from a drive-to-pose
            DriveToCommand.cancel(); // MJD
            bAutoDrive = false; // MJD
        } // MJD

        if (bAutoDrive) {
            //if((CommonLogic.DeadBand(gamepad1.left_stick_x) != 0.0) || (CommonLogic.DeadBand(gamepad1.left_stick_y) != 0.0) || (CommonLogic.DeadBand(gamepad1.right_stick_x) != 0.0)){   // MJD: old, broke out instantly
            // MJD: sticks only break out once the delay has passed, so you can press A/B/X/Y with the sticks still held
            if(autoDriveTimer.seconds() >= AUTO_DRIVE_BREAKOUT_DELAY_SEC && ((CommonLogic.DeadBand(gamepad1.left_stick_x) != 0.0) || (CommonLogic.DeadBand(gamepad1.left_stick_y) != 0.0) || (CommonLogic.DeadBand(gamepad1.right_stick_x) != 0.0))){ // MJD
                DriveToCommand.cancel();
             bAutoDrive = false;
            }

       


        }else if (alignHeld && robot.limelight.hasClosestTag()) { // MJD: sticks still drive, the turn is automatic
            robot.driveTrain.cmdTeleOpAutoTurn( // MJD
                    CommonLogic.joyStickMath(gamepad1.left_stick_y * -1), // MJD
                    CommonLogic.joyStickMath(gamepad1.left_stick_x * -1), // MJD
                    //robot.limelight.getAlignTurnPower(robot.driveTrain.getPose()), // MJD: old, moved to AutoAlign
                    robot.autoAlign.getTurnPower(robot.driveTrain.getPose(), robot.limelight.getClosestTagField()), // MJD
                    driveSpeed); // MJD
        }else{


                robot.driveTrain.cmdTeleOp(
                        CommonLogic.joyStickMath(gamepad1.left_stick_y * -1),
                        CommonLogic.joyStickMath(gamepad1.left_stick_x * -1),
                        CommonLogic.joyStickMath(gamepad1.right_stick_x * -1),
                        driveSpeed
                );


        }

        gp1_prev_a = gamepad1.a;
        // MJD: gamepad 1 right bumper toggles the flower ramp (press = up, press again = down)
        if (CommonLogic.oneShot(gamepad1.right_bumper, gp1_prev_right_bumper)) { // MJD
            if (robot.flower_ramp.CurrentMode == Flower_Ramp.Mode.Up) { // MJD
                robot.flower_ramp.cmdDown(); // MJD
            } else { // MJD
                robot.flower_ramp.cmdUp(); // MJD
            } // MJD
        } // MJD
        gp1_prev_right_bumper = gamepad1.right_bumper; // MJD
        gp1_prev_b = gamepad1.b;
        gp1_prev_y = gamepad1.y;
        gp1_prev_x = gamepad1.x;

        //gamepad1
        // CHANGED: gamepad 1 is driving and speed changes only for now. The launcher, blocker,
        // feeder and intake controls moved to gamepad 2. The old gamepad 1 lines are commented
        // out below so they are easy to restore.

        // CHANGED: the Comp bot has no TransitionR servo, so every transitionR line is commented out.
        /*if ((gamepad1.b)) {
            //robot.intake.cmdIntakeForward();
            //robot.transitionR.cmdSpinng();

        }

        if ((gamepad1.a)) {
            //robot.intake.cmdIntakeStop();
            //robot.transitionR.cmdStop();

        }

        if ((gamepad1.y)) {
            //robot.intake.cmdIntakeReverse();
            //robot.transitionR.cmdTumble();
        }

         */

        // TODO (reminder): Launcher now has state tracking. Still to do here:
        //  1. Map a button to robot.feeder.cmdFeederBackward() (no button assigned yet).
        //  2. Replace the old commented-out "bAtSpeed" gate below with
        //     robot.launcher.isLauncherAtSpeed() / robot.feeder.isFeederAtSpeed() before feeding.
        //  3. Optionally show getLauncherState() / getFeederState() in debug telemetry.
        // DONE (1): gamepad2.dpad_left runs the feeder backward while held.
        // DONE (2): the shoot block (now on gamepad2.right_trigger, further down) gates the
        //           blocker and feeder on isLauncherAtSpeed().
        // DONE (3): the Launcher and Feeder classes print their own states; the debug block adds the at-speed flags.
        // CHANGED: the shoot block moved to gamepad 2. The old gamepad 1 version is kept here, commented out.

      boolean shootHeld1 = gamepad1.right_trigger_pressed; // MJD

        if (CommonLogic.oneShot(shootHeld1, gp1_prev_right_trigger)) { // MJD
            robot.tripleThreat.cmdShoot();          // once, when the trigger is first pressed // MJD
        } else if (CommonLogic.oneShotRelease(shootHeld1, gp1_prev_right_trigger)) { // MJD
            robot.tripleThreat.cmdCeaseFire();      // once, when it is released // MJD
        } // MJD
        gp1_prev_right_trigger = shootHeld1; // MJD


/*

        } else if (gp1_prev_right_trigger) {
            robot.pollenBlocker.cmdBlock();
            //robot.transitionR.cmdStop();
            robot.feeder.cmdFeederStop();
        }
        gp1_prev_right_trigger = shootHeld;
        */

        if (gamepad1.left_trigger_pressed) {

        }

        // CHANGED: dpad left/up/down now set the drive speed. They used to call
        // cmdTeleOp(0, 0, 0, ...), which zeroed the sticks while the button was held.
        if (gamepad1.dpad_left) {
            driveSpeed = robot.driveTrain.NORMALSPEED;
        }

        if (gamepad1.dpad_right) {

        }

        if ((gamepad1.dpad_up)){
            driveSpeed = robot.driveTrain.FASTSPEED;

        }

        if ((gamepad1.dpad_down)){
            driveSpeed = robot.driveTrain.MORNINGSPEED;

        }

        if ((gamepad1.x)) {

        }



        if ((gamepad1.back)){
            robot.driveTrain.resetDriveHeading();

        }

//       if ((gamepad1.left_bumper)) {   // MJD: old, this faked a "Y released" every loop and stopped the feeder/intake
//           intakeReversePrev = true;
//
//       }



        //gamepad2

        if ((gamepad2.b)) {
            robot.intake.cmdIntakeForward();
            robot.overrideIntakeFull(); // MJD: driver wants the intake on even if the sensors say full
            //robot.transitionR.cmdSpinng();

        }

        if ((gamepad2.a)) {
            robot.intake.cmdIntakeStop();

            //robot.transitionR.cmdStop();

        }

//        if ((gamepad1.y)) {   // MJD: old, reverse latched on after one press
//            robot.intake.cmdIntakeReverse();
//            //robot.transitionR.cmdTumble();
//
//        }

        // MJD: reverse only while the button is held (gamepad 2 Y). On release the intake goes back to
        // what it was doing (forward if it was forward, otherwise stopped).
        //boolean reverseHeld = gamepad2.y;// MJD: old, gamepad 2 only
        boolean reverseHeld = gamepad2.y || gamepad1.left_bumper;// MJD: gamepad 2 Y or gamepad 1 left bumper
        if (CommonLogic.oneShot(reverseHeld, intakeReversePrev)) { // MJD
            intakeWasForward = robot.intake.isIntakeForward(); // MJD
        } // MJD
        if (reverseHeld) { // MJD
            robot.intake.cmdIntakeReverse(); // MJD
            robot.feeder.cmdFeederBackward();
        } else if (CommonLogic.oneShotRelease(reverseHeld, intakeReversePrev)) { // MJD
            if (intakeWasForward) { // MJD
                robot.intake.cmdIntakeForward();// MJD
                robot.feeder.cmdFeederStop();
            } else { // MJD
                robot.intake.cmdIntakeStop(); // MJD
                robot.feeder.cmdFeederStop();
            } // MJD
        } // MJD
        intakeReversePrev = reverseHeld; // MJD



        // CHANGED: x is now a full stop: launcher off, feeder off, blocker closed,
        // and any shot in progress is cancelled. (Before it left the blocker where it was.)
        if ((gamepad2.x)) {
            robot.tripleThreat.cmdTripleThreatStop();
            //robot.launcher.cmdLauncherStop();
            //robot.feeder.cmdFeederStop();
            //volleyActive = false;
        }


        if (gamepad2.right_bumper) {
            robot.launcher.cmdLauncherForward();
        }

        // CHANGED: the manual feeder controls only act while TripleThreat is idle,
        // so they can't fight the launch sequence.
        if (gamepad2.left_bumper && robot.tripleThreat.isIdle()) {
            robot.feeder.cmdFeederForward();
        }

        // CHANGED: feeder backward while held (the TODO "map a button to cmdFeederBackward").
        if (gamepad2.dpad_left) {
        }

        if (gamepad2.dpad_right) {

        }

        if ((gamepad2.dpad_up)){
            robot.flower_ramp.cmdUp();


        }
        if ((gamepad2.dpad_down)){
            robot.flower_ramp.cmdDown();
        }

        // CHANGED: manual blocker override. It only acts while the trigger is held (open) and
        // once on release (close), and only while TripleThreat is idle. Before, the else branch
        // ran every loop and kept closing the blocker, which cancelled the shoot sequence.
        boolean  blockerOverrideHeld = gamepad2.left_trigger > 0.6;

        if (robot.tripleThreat.isIdle()) {
            if (blockerOverrideHeld){
                robot.pollenBlocker.cmdOpen();

            }else if (gp2_prev_left_trigger){
                robot.pollenBlocker.cmdBlock();
            }
        }
        gp2_prev_left_trigger = blockerOverrideHeld;


        // CHANGED: the whole shoot sequence is one call now. Hold the trigger: TripleThreat spins up
        // the launcher, waits for speed, starts the feeder, then opens the blocker and keeps firing.
        // Release it: the blocker closes and the feeder stops (the launcher keeps spinning for the
        // next volley; x stops it).
        // old shoot block commented out (cmdCeaseFire ran every loop while held, which cancelled the sequence)
        boolean shootHeld = gamepad2.right_trigger_pressed; // MJD

        if (CommonLogic.oneShot(shootHeld, gp2_prev_right_trigger)) { // MJD
            robot.tripleThreat.cmdShoot();          // once, when the trigger is first pressed // MJD
        } else if (CommonLogic.oneShotRelease(shootHeld, gp2_prev_right_trigger)) { // MJD
            robot.tripleThreat.cmdCeaseFire();      // once, when it is released // MJD
        } // MJD
        gp2_prev_right_trigger = shootHeld; // MJD

//        boolean shootHeld = gamepad2.right_trigger_pressed;

//        if (shootHeld) {
//            if(!shootAborted){
//                robot.tripleThreat.cmdShoot();
//            }
//            if(robot.tripleThreat.didLastShotAbort()){
//                shootAborted = true;
//            }
//            else{
//                shootAborted = false;
//            }
//            if(gp2_prev_right_trigger){
//                robot.tripleThreat.cmdCeaseFire();
//            }
//            gp2_prev_right_trigger = shootHeld;
//            /* old latch version, replaced by the TripleThreat state machine:
//            robot.launcher.cmdLauncherForward();

//            // CHANGED: the at-speed check only STARTS the volley. After that, dips in RPM
//            // (a ball loading the flywheel) no longer close the blocker.
//            if (!volleyActive && robot.launcher.isLauncherAtSpeed()) {
//                volleyActive = true;
//            }

//            if(volleyActive){
//                robot.pollenBlocker.cmdOpen();
//                robot.feeder.cmdFeederForward();


//            }else{
//                robot.pollenBlocker.cmdBlock();
//                robot.feeder.cmdFeederStop();

//            }
//            */



//        } else if (gp2_prev_right_trigger) {
//            robot.tripleThreat.cmdCeaseFire();
//            //volleyActive = false;
//            //robot.pollenBlocker.cmdBlock();
//            //robot.feeder.cmdFeederStop();
//        }
//        gp2_prev_right_trigger = shootHeld;





        boolean inLaunchPos = InLaunchPosition(); // MJD
        if (!bAutoDrive && !inLaunchPos) { // MJD: driver took over and left the spot, back to the normal speed
            poseDriveCalled = false; // MJD
        } // MJD
        //robot.launcher.setPoseSpeedMode(poseDriveCalled && inLaunchPos); // MJD: old, pose speed only once the light was green
        robot.launcher.setPoseSpeedMode(poseDriveCalled); // MJD: pose speed from the button press until the driver takes over and leaves the spot
        //robot.lighting.cmdInLaunchPosition(InLaunchPosition());   // MJD: old
        robot.lighting.cmdInLaunchPosition(inLaunchPos); // MJD
        Scheduler.execute();
        // robot.loop() -> driveTrain.loop() -> follower.update(), which sends
        // the power to the motors. Must come AFTER cmdTeleOp.
        robot.loop();
        PanelsTelemetry.INSTANCE.getTelemetry().update();
        // ADDED: debug telemetry so you can see what's happening.
        if (debug) {
            Pose p = robot.driveTrain.getPose();
            telemetry.addData("Stick fwd/strafe/turn", "%.2f / %.2f / %.2f",
                    -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            telemetry.addData("Robot X (in)", "%.1f", p.x());
            telemetry.addData("Robot Y (in)", "%.1f", p.y());
            telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(p.heading()));
            telemetry.addData("Battery Voltage","%.2f", robot.launcher.getBatteryVoltage());
            // CHANGED: at-speed flags and blocker state (the parts already print their own states).
            telemetry.addData("Launcher at speed", robot.launcher.isLauncherAtSpeed());
            telemetry.addData("Feeder at speed", robot.feeder.isFeederAtSpeed());
            telemetry.addData("Blocker open", robot.pollenBlocker.isOpen());
            telemetry.addData("Pose launch speed on", poseDriveCalled); // MJD
            telemetry.addData("Align error (deg)", "%.1f", robot.autoAlign.getErrorDeg()); // MJD
            telemetry.addData("Aligned", robot.autoAlign.isAligned()); // MJD
            telemetry.addData("bAutoDrive", bAutoDrive);
            telemetry.addData("Hold cmd scheduled", DriveToCommand != null && DriveToCommand.isScheduled());
            telemetry.addData("Follower busy", robot.driveTrain.isBusy());
            telemetry.addData("Follower mode", robot.driveTrain.getModeText());
            if (lastTarget != null) {
                telemetry.addData("Dist to target (in)", "%.1f",
                        Math.hypot(p.x() - lastTarget.x(), p.y() - lastTarget.y()));
            }
            //telemetry.addData("Volley active", volleyActive);   // CHANGED: replaced by "TripleThreat State" from the TripleThreat class


        }
        telemetry.update();
    }

    @Override
    public void stop() {
        // ADDED: without this, DriveTrain.stop() never saves the pose.
        robot.stop();
        Scheduler.reset();
    }

}