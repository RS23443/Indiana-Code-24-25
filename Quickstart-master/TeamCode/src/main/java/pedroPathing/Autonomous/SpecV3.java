package pedroPathing.Autonomous;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import pedroPathing.Robot.Commands.PDFLCommand;
import pedroPathing.Robot.Constants;
import pedroPathing.Robot.PIDController;
import pedroPathing.Robot.Sensing.PDFL;
import pedroPathing.Robot.Sensing.SleepCode;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import pedroPathing.Robot.Systems.*;
import com.pedropathing.localization.Pose;


/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.2, 6/10/2025
 */

@Autonomous(name = "Spec 3", group = "Autonomous")
@Disabled
public class SpecV3 extends OpMode {
    // FTC-GEARHEADS: Increased Y-Offset to provide more clearance from the truss
    private double YOffset = 10;

    private Follower follower;
    private Intake intake;
    private Deposit deposit;
    private Lifts lifts;
    private VoltageSensor voltageSensor;

    private Timer pathTimer, actionTimer, opmodeTimer, AutonTimer;
    private PDFL pdfLController;
    private PIDController pidController;

    // Tuning variables accessible via FTC Dashboard
    public double kPi = 0.01, kIi = 0.00, kDi = 0.000, alphai = 0.075;

    private double kP = 0.008, kD = 0.003, kF = 0.133, kL = 0.1;
    private double deadzone = 25.0;
    private double homedConstant = 0;
    private int target;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    // Poses
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));

    // FTC-GEARHEADS: Adjusted scoring and grabbing poses for smoother, curved paths
    private final Pose scorePose1 = new Pose(31, 10 + YOffset, Math.toRadians(315)); // Angled for better approach
    private final Pose scorePose2 = new Pose(36, 14 + YOffset, Math.toRadians(315));
    private final Pose scorePose3 = new Pose(38, 12 + YOffset, Math.toRadians(315));
    private final Pose parkPose = new Pose(7, -35 + YOffset, Math.toRadians(135));

    private final Pose pickup1Pose = new Pose(53, -27 + YOffset, Math.toRadians(0));
    private final Pose pickup1Pose2 = new Pose(10, -28 + YOffset, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(53, -36 + YOffset, Math.toRadians(0));
    private final Pose pickup2Pose2 = new Pose(7, -40 + YOffset, Math.toRadians(0));
    private final Pose grabSpecPose = new Pose(7, -28.5 + YOffset, Math.toRadians(0));

    // Paths
    private PathChain grabPickup1, grabPickup12, grabPickup2, grabPickup22;
    // FTC-GEARHEADS: Consolidated multiple line paths into single, fluid curves
    private PathChain scoreCycle1, scoreCycle2, scoreCycle3;
    private Path park;


    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths) **/
    public void buildPaths() {
        // Initial pickup paths remain largely the same
        grabPickup1 = follower.pathBuilder()
                // FTC-GEARHEADS FIX: Corrected Point constructor to use Point.CARTESIAN identifier instead of a heading.
                .addPath(new BezierCurve(new Point(startPose), new Point(22,-9+YOffset, Point.CARTESIAN), new Point(39,1+YOffset, Point.CARTESIAN),new Point(pickup1Pose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), pickup1Pose.getHeading())
                .build();

        grabPickup12 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(pickup1Pose2)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), pickup1Pose2.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                // FTC-GEARHEADS FIX: Corrected Point constructor
                .addPath(new BezierCurve(new Point(pickup1Pose2), new Point(45, -29+YOffset, Point.CARTESIAN),new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(pickup1Pose2.getHeading(), pickup2Pose.getHeading())
                .build();

        grabPickup22 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(pickup2Pose2)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), pickup2Pose2.getHeading())
                .build();

        // FTC-GEARHEADS: REFACTORING PATHS FOR FLUIDITY
        // Path for the first score-and-grab cycle
        scoreCycle1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(pickup2Pose2), // Start from the last pickup spot
                        new Point(18, -10 + YOffset, Point.CARTESIAN), // Control point to shape the curve
                        new Point(scorePose1) // End at the first scoring position
                ))
                .setTangentHeadingInterpolation()
                .addPath(new BezierCurve(
                        new Point(scorePose1), // Start from scoring position
                        new Point(15, -18 + YOffset, Point.CARTESIAN), // Control point
                        new Point(grabSpecPose) // End at the next grab position
                ))
                .setTangentHeadingInterpolation()// KEY CHANGE: Ensures smooth, fast diagonal movement
                .build();

        // Path for the second score-and-grab cycle
        scoreCycle2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(grabSpecPose),
                        new Point(20, -5 + YOffset, Point.CARTESIAN),
                        new Point(scorePose2)
                ))
                .addPath(new BezierCurve(
                        new Point(scorePose2),
                        new Point(18, -16 + YOffset, Point.CARTESIAN),
                        new Point(grabSpecPose) // Re-using the same grab pose
                ))
                .setTangentHeadingInterpolation()
                .build();

        // Path for the third score-and-grab cycle
        scoreCycle3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(grabSpecPose),
                        new Point(22, -3 + YOffset, Point.CARTESIAN),
                        new Point(scorePose3)
                ))
                .setTangentHeadingInterpolation()
                .build();


        /* This is our park path. */
        park = new Path(new BezierLine(new Point(scorePose3), new Point(parkPose)));
        park.setLinearHeadingInterpolation(scorePose3.getHeading(), parkPose.getHeading());
    }

    /** This switch is called continuously and runs the pathing. **/
    public void autonomousPathUpdate() {
        // FTC-GEARHEADS: Simplified the state machine to match the new, consolidated paths.
        switch (pathState) {
            case 0: // Initial pickups
                follower.followPath(grabPickup1);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()){
                    follower.followPath(grabPickup12);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()){
                    follower.followPath(grabPickup2);
                    setPathState(3);
                }
                break;
            case 3:
                if(follower.isBusy()){
                    deposit.setServoPosition(1, Constants.outtakeSpecimenReset[0]);
                    deposit.setServoPosition(2,Constants.outtakeSpecimenReset[1]);
                    deposit.setServoPosition(3,Constants.outtakeSpecimenReset[2]);
                    deposit.setServoPosition(4,Constants.outtakeSampleReset[3]);
                }
                if(!follower.isBusy()){
                    follower.followPath(grabPickup22);
                    setPathState(4);
                }
                break;
            case 4: // Start of the first scoring cycle
                if(!follower.isBusy()){
                    // Actions to prepare for scoring
                    deposit.setServoPosition(4,Constants.outtakeSpecimenDrop[3]);
                    SleepCode(0.15);
                    loopLifts(320,0.2,20);
                    deposit.setServoPosition(1,0.26);
                    deposit.setServoPosition(2,0.74);
                    deposit.setServoPosition(3,0.35);

                    // Follow the new, smooth path
                    follower.followPath(scoreCycle1, true);
                    setPathState(5);
                }
                break;
            case 5: // First cycle is done, start the second
                if(!follower.isBusy()){
                    // Reset/Grab actions
                    loopLifts(1000,0.5,20);
                    deposit.setServoPosition(4,Constants.outtakeSampleReset[3]);
                    loopLifts(0,0.2,20);

                    // Prepare for scoring again
                    deposit.setServoPosition(4,Constants.outtakeSpecimenDrop[3]);
                    SleepCode(0.15);
                    loopLifts(320,0.2,20);
                    deposit.setServoPosition(1,0.26);
                    deposit.setServoPosition(2,0.74);
                    deposit.setServoPosition(3,0.35);

                    follower.followPath(scoreCycle2, true);
                    setPathState(6);
                }
                break;
            case 6: // Second cycle is done, start the third
                if(!follower.isBusy()){
                    // Reset/Grab actions
                    loopLifts(1000,0.35,20);
                    deposit.setServoPosition(4,Constants.outtakeSampleReset[3]);
                    loopLifts(0,0.2,20);

                    // Prepare for scoring a final time
                    deposit.setServoPosition(4,Constants.outtakeSpecimenDrop[3]);
                    SleepCode(0.15);
                    loopLifts(320,0.2,20);
                    deposit.setServoPosition(1,0.26);
                    deposit.setServoPosition(2,0.74);
                    deposit.setServoPosition(3,0.35);

                    follower.followPath(scoreCycle3, true);
                    setPathState(7);
                }
                break;
            case 7: // Final cycle is done, park the robot
                if(!follower.isBusy()){
                    // Final reset actions
                    loopLifts(1000,0.35,20);
                    deposit.setServoPosition(4,Constants.outtakeSampleReset[3]);
                    loopLifts(0,0.2,20);
                    follower.followPath(park);
                    setPathState(8); // Move to a finished state
                }
                break;
            case 8:
                // Autonomous finished
                break;
        }
    }

    /** These change the states of the paths and actions **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        lifts.periodic();
        boolean stop = AutonTimeChecker();

        if(stop){
            setPathState(-1);
        }

        if(intake.getExtensionPosition() >= 0){
            intake.setMotorPower(-0.1);
        }

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading (Degrees)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();
        AutonTimer = new Timer();
        opmodeTimer.resetTimer();

        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        intake = new Intake(hardwareMap);
        deposit = new Deposit(hardwareMap);
        lifts = new Lifts(hardwareMap, voltageSensor.getVoltage());
        follower.setStartingPose(startPose);

        intake.setServoPosition(1, 0.6);
        intake.setServoPosition(2, 0.4);
        intake.setServoPosition(3,0.5);
        intake.setServoPosition(4,0.5);
        intake.setServoPosition(6, 0.7);

        deposit.setServoPosition(1,0.45);
        deposit.setServoPosition(2,0.55);
        deposit.setServoPosition(3,0.35);
        deposit.setServoPosition(4,Constants.outtakeSampleDrop[3]);
        pdfLController = new PDFL(kP, kD, kF, kL);
        pdfLController.setDeadzone(deadzone);
        pdfLController.setHomedConstant(homedConstant);
        pidController = new PIDController(kPi,kIi,kDi,alphai);

        buildPaths();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
        intake.setMotor(0);
    }

    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {
        AutonTimer.resetTimer();
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }

    public void SleepCode(double seconds){
        actionTimer.resetTimer();
        while(actionTimer.getElapsedTimeSeconds() < seconds){
            //This is a blocking call, be careful with it in loops
        }
    }

    public void PDFL(int target){
        int pos = (int)lifts.getTopMotorData()[0];
        int direction = target-pos;
        if(direction < 0 && pos >= target + deadzone) {
            pdfLController.updateConstants(kP, kD, kF, kL);
            pdfLController.setDeadzone(deadzone);
            pdfLController.setHomedConstant(homedConstant);
            pos = (int) lifts.getTopMotorData()[0];
            double error2 = target - pos;
            double powerTop = pdfLController.run(error2);
            powerTop = Math.max(-1, Math.min(powerTop, 1));
            lifts.SetPower(powerTop);
        } else if (direction > 0 && pos <= target -deadzone){
            pdfLController.updateConstants(kP, kD, kF, kL);
            pdfLController.setDeadzone(deadzone);
            pdfLController.setHomedConstant(homedConstant);
            pos = (int) lifts.getTopMotorData()[0];
            double error2 = target - pos;
            double powerTop = pdfLController.run(error2);
            powerTop = Math.max(-1, Math.min(powerTop, 1));
            lifts.SetPower(powerTop);
        } else {
            lifts.SetPower(kF/voltageSensor.getVoltage());
        }
    }

    public void loopLifts(int target, double totalTimeinSeconds, int numberOfLoops){
        for (int i = 0; i < numberOfLoops; i++) {
            PDFL(target);
            SleepCode(totalTimeinSeconds / numberOfLoops);
        }
        lifts.SetPower(kF);
    }

    public void PIDIntake(int target, double totalTimeinSeconds, int numberOfLoops){
        for (int i = 0; i < numberOfLoops; i++) {
            double power = pidController.compute(intake.getExtensionPosition(),target);
            intake.setMotorPower(power);
            SleepCode(totalTimeinSeconds / numberOfLoops);
        }
        intake.setMotorPower(0.075);

    }

    public boolean AutonTimeChecker(){
        double timePassed = AutonTimer.getElapsedTimeSeconds();
        if(timePassed >= 32){
            return true;
        }
        return false;
    }
}
