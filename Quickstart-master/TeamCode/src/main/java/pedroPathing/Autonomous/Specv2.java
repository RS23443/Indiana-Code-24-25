package pedroPathing.Autonomous;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
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

import pedroPathing.Robot.Commands.DepoResetAuto;
import pedroPathing.Robot.Commands.DepositDropPositionCommand;
import pedroPathing.Robot.Commands.DepositResetCommand;
import pedroPathing.Robot.Commands.Intake45Degrees;
import pedroPathing.Robot.Commands.IntakeActiveCommand;
import pedroPathing.Robot.Commands.IntakeExtensionControlCommand;
import pedroPathing.Robot.Commands.IntakeGrabAuto;
import pedroPathing.Robot.Commands.IntakeGrabCommand;
import pedroPathing.Robot.Commands.PDFLCommand;
import pedroPathing.Robot.Commands.SampleSwitchCommand;
import pedroPathing.Robot.Commands.TotalIntakeMovement;
import pedroPathing.Robot.Constants;
import pedroPathing.Robot.PIDController;
import pedroPathing.Robot.Sensing.PDFL;
import pedroPathing.Robot.Sensing.SleepCode;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import pedroPathing.Robot.Systems.*;


/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

@Autonomous(name = "Spec 2", group = "Autonomous")
@Disabled
public class Specv2 extends OpMode {
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

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePose = new Pose(31, 10+YOffset, Math.toRadians(0));
    private final Pose scorePose2 = new Pose(36,14+YOffset,Math.toRadians(0));
    private final Pose scorePose3 = new Pose(38,12+YOffset, Math.toRadians(0));
    private final Pose scorePose4 = new Pose(31,18+YOffset,Math.toRadians(0));
    private final Pose scoreControl = new Pose(8,10+YOffset,Math.toRadians(270));
    private final Pose scoreControl2 = new Pose(25,-3+YOffset,Math.toRadians(180));
    private final Pose scoreControl3 = new Pose(25,9+YOffset,Math.toRadians(0));

    /** Lowest (First) Sample from the Spike Mark */
    private final Pose pickup1Pose = new Pose(53, -27+YOffset, Math.toRadians(0));
    private final Pose pickUpPose1Control = new Pose(22,-9+YOffset,Math.toRadians(0));
    private final Pose pickUpPose1Control2 = new Pose(39,1+YOffset,Math.toRadians(0));

    private final Pose pickup1Pose2 = new Pose(10,-28+YOffset,Math.toRadians(0));

    /** Middle (Second) Sample from the Spike Mark */
    private final Pose pickup2Pose = new Pose(53, -36+YOffset, Math.toRadians(0));
    private final Pose pickup2PoseControl = new Pose(45, -29+YOffset, Math.toRadians(0));
    private final Pose pickup2Pose2 = new Pose(7, -40+YOffset, Math.toRadians(0));

    /** Highest (Third) Sample from the Spike Mark */
    private final Pose grabSpecPose = new Pose(7, -28.5+YOffset, Math.toRadians(0));
    private final Pose grabSpecPose3 = new Pose(10, -28.5+YOffset, Math.toRadians(0));


    /** Park Pose for our robot, after we do all of the scoring. */
    private final Pose parkPose = new Pose(7, -35+YOffset, Math.toRadians(135));

    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve. */

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path park;
    private PathChain grabPickup1, grabPickup12, grabPickup2, grabPickup22, grabSpec1, grabspec2, grabspec3, scorespec1, scorespec2,scorespec3,scorespec4;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         *    * BezierLines are straight, and require 2 points. There are the start and end points.
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
         *    * Constant Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(startPose), new Point(pickUpPose1Control), new Point(pickUpPose1Control2),new Point(pickup1Pose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), pickup1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup12 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(pickup1Pose2)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), pickup1Pose2.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickup1Pose2), new Point(pickup2PoseControl),new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(pickup1Pose2.getHeading(), pickup2Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup22 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(pickup2Pose2)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), pickup2Pose2.getHeading())
                .build();
        scorespec1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose2),new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup2Pose2.getHeading(), scorePose.getHeading())
                .build();
        grabSpec1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose),new Point(grabSpecPose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), grabSpecPose.getHeading())
                .build();
        scorespec2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(grabSpecPose),new Point(scorePose2)))
                .setLinearHeadingInterpolation(grabSpecPose.getHeading(), scorePose2.getHeading())
                .build();
        grabspec2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose2),new Point(grabSpecPose3)))
                .setLinearHeadingInterpolation(scorePose2.getHeading(), grabSpecPose3.getHeading())
                .build();
        scorespec3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(grabSpecPose3),new Point(scorePose3)))
                .setLinearHeadingInterpolation(grabSpecPose3.getHeading(), scorePose3.getHeading())
                .build();



        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */

        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
        park = new Path(new BezierLine(new Point(scorePose3), new Point(parkPose)));
        park.setLinearHeadingInterpolation(scorePose4.getHeading(), parkPose.getHeading());
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
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
            case 4:
                if(!follower.isBusy()){
                    deposit.setServoPosition(4,Constants.outtakeSpecimenDrop[3]);
                    SleepCode(0.15);
                    loopLifts(320,0.2,20);

                    deposit.setServoPosition(1,0.26);
                    deposit.setServoPosition(2,0.74);
                    deposit.setServoPosition(3,0.35);
                    follower.followPath(scorespec1,true);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()){
                    loopLifts(1000,0.5,20);
                    deposit.setServoPosition(4,Constants.outtakeSampleReset[3]);
                    loopLifts(0,0.2,20);
                    follower.followPath(grabSpec1);
                    setPathState(6);
                }
                break;
            case 6:
                if(follower.isBusy()){
                    deposit.setServoPosition(1, Constants.outtakeSpecimenReset[0]);
                    deposit.setServoPosition(2,Constants.outtakeSpecimenReset[1]);
                    deposit.setServoPosition(3,Constants.outtakeSpecimenReset[2]);
                    deposit.setServoPosition(4,Constants.outtakeSampleReset[3]);
                }
                if(!follower.isBusy()){
                    follower.turnDegrees(45,true);
                    deposit.setServoPosition(4,Constants.outtakeSpecimenDrop[3]);
                    SleepCode(0.15);
                    loopLifts(320,0.2,20);
                    deposit.setServoPosition(1,0.26);
                    deposit.setServoPosition(2,0.74);
                    deposit.setServoPosition(3,0.35);
                    follower.followPath(scorespec2,true);
                    setPathState(7);
                }

                break;
            case 7:
                if(!follower.isBusy()){
                    loopLifts(1000,0.35,20);
                    deposit.setServoPosition(4,Constants.outtakeSampleReset[3]);
                    loopLifts(0,0.2,20);
                    follower.followPath(grabspec2);
                    setPathState(8);
                }

                break;
            case 8:
                if(follower.isBusy()){
                    deposit.setServoPosition(1, Constants.outtakeSpecimenReset[0]);
                    deposit.setServoPosition(2,Constants.outtakeSpecimenReset[1]);
                    deposit.setServoPosition(3,Constants.outtakeSpecimenReset[2]);
                    deposit.setServoPosition(4,Constants.outtakeSampleReset[3]);
                }
                if(!follower.isBusy()){
                    deposit.setServoPosition(4,Constants.outtakeSpecimenDrop[3]);
                    SleepCode(0.15);
                    loopLifts(320,0.2,20);
                    deposit.setServoPosition(1,0.26);
                    deposit.setServoPosition(2,0.74);
                    deposit.setServoPosition(3,0.35);
                    follower.followPath(scorespec3,true);
                    setPathState(9);
                }

                break;
            case 9:
                if(!follower.isBusy()){
                    loopLifts(1000,0.35,20);
                    deposit.setServoPosition(4,Constants.outtakeSampleReset[3]);
                    loopLifts(0,0.2,20);
                    follower.followPath(park);
                    setPathState(10);
                }

                break;

        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
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
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
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

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
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
            telemetry.addLine("Sleep Code is Running");
            telemetry.update();
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

