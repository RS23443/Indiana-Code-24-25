package org.firstinspires.ftc.teamcode.OpModes.Autonomous; // Updated package

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive; // Updated drive class
import org.firstinspires.ftc.teamcode.Robot.Camera.Pipelines.ContrastAndROI; // Updated vision import
import org.firstinspires.ftc.teamcode.Robot.Constants;
import org.firstinspires.ftc.teamcode.Robot.Systems.Intake;
import org.firstinspires.ftc.teamcode.Robot.Systems.Lifts;
import org.firstinspires.ftc.teamcode.Robot.Systems.Deposit;


import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Four Sample with attempted vision", group = "Autonomous")
// @Disabled
public class FourSample extends LinearOpMode {

    MecanumDrive drive;
    public VoltageSensor controlHubVoltageSensor;

    OpenCvCamera webcam;
    ContrastAndROI visionPipeline;
    DetectionResult detectedPosition = DetectionResult.NONE;
    int cameraWidth = 640; int cameraHeight = 480;
    Rect inspectionROI;
    private volatile boolean isWebcamStreaming = false;
    private static final double SCAN_DURATION_SEC = 1;
    private ElapsedTime scanTimer = new ElapsedTime();
    private static final int MAX_SCAN_ATTEMPTS = 3;
    // private static final double INCREMENTAL_MOVE_DISTANCE = 6.0; // Drivetrain move, replaced by intake extension

    public Intake intake;
    public Lifts lifts;
    public Deposit deposit;

    public enum DetectionResult { LEFT, CENTER, RIGHT, NONE }

    // --- Action Classes (Assumed to be mostly correct from previous version) ---
    public class SetIntakeServosAction implements Action {
        private final double[] targetPositions;
        private boolean initialized = false;
        private static final double SERVO_ACTION_DURATION = 0.4;
        private ElapsedTime timer = new ElapsedTime();
        public SetIntakeServosAction(double[] positions) { this.targetPositions = positions; }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                intake.setServoPosition(1, targetPositions[0]);
                intake.setServoPosition(2, targetPositions[1]);
                intake.setServoPosition(3, targetPositions[2]);
                intake.setServoPosition(4, targetPositions[3]);
                intake.setServoPosition(5, targetPositions[4]);
                timer.reset(); initialized = true;
                packet.put("IntakeServosAction", "Claw to " + targetPositions[4]);
            }
            if (timer.seconds() > SERVO_ACTION_DURATION) return false;
            return true;
        }
    }

    public class MoveIntakeExtensionPIDAction implements Action {
        private final int targetTicks;
        private boolean initialized = false;
        private ElapsedTime timeoutTimer = new ElapsedTime();
        private static final double EXTENSION_TIMEOUT_SEC = 2.5;
        private static final int POSITION_TOLERANCE = 20;

        public MoveIntakeExtensionPIDAction(int targetTicks) { this.targetTicks = targetTicks; }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!opModeIsActive()) { intake.setMotorPower(0); return false;}
            if (!initialized) {
                telemetry.addData("IntakeExtPIDAction: Moving to", targetTicks);
                initialized = true; timeoutTimer.reset();
            }
            intake.setMotor(targetTicks); // Calls PID
            int currentPos = intake.getExtensionPosition();
            boolean atTarget = Math.abs(currentPos - targetTicks) < POSITION_TOLERANCE;
            packet.put("IntakeExt Target", targetTicks);
            packet.put("IntakeExt CurrentPos", currentPos);
            if (atTarget) {
                intake.setMotorPower(0.05 * Math.signum(targetTicks - currentPos));
                if(Math.abs(targetTicks - currentPos) < 5) intake.setMotorPower(0);
                return false;
            }
            if (timeoutTimer.seconds() > EXTENSION_TIMEOUT_SEC) {
                telemetry.addLine("IntakeExtPIDAction: Timed out.");
                intake.setMotorPower(0);
                return false;
            }
            return true;
        }
    }

    public class MoveLiftPDFLAction implements Action {
        private final int targetTicks;
        private boolean initialized = false;
        private ElapsedTime timeoutTimer = new ElapsedTime();
        private static final double LIFT_TIMEOUT_SEC = 3.5;
        private static final int POSITION_TOLERANCE = 25;

        public MoveLiftPDFLAction(int targetTicks) { this.targetTicks = targetTicks; }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!opModeIsActive()) { lifts.enablePDControl(false); lifts.stopSlides(); return false; }
            if (!initialized) {
                lifts.setTarget(targetTicks);
                lifts.enablePDControl(true);
                initialized = true; timeoutTimer.reset();
                packet.put("LiftPDFLAction", "Init to " + targetTicks);
            }
            lifts.updatePDControl();
            double currentTopPos = lifts.getTopMotorData()[0];
            double middleMotorVel = lifts.getMiddleMotorData()[1];
            double bottomMotorVel = lifts.getBottomMotorData()[1];
            double topMotorVel = lifts.getTopMotorData()[1];
            boolean positionReached = Math.abs(currentTopPos - targetTicks) < POSITION_TOLERANCE;
            boolean anyMotorEffectivelyStopped = Math.abs(middleMotorVel) < 10 && Math.abs(bottomMotorVel) < 10 && Math.abs(topMotorVel) < 10;

            packet.put("LiftPDFL Target", targetTicks);
            packet.put("LiftPDFL TopPos", String.format("%.1f", currentTopPos));

            if (positionReached && anyMotorEffectivelyStopped) {
                lifts.runslides(lifts.DynamicKF()); lifts.enablePDControl(false);
                return false;
            }
            if (timeoutTimer.seconds() > LIFT_TIMEOUT_SEC) {
                lifts.runslides(lifts.DynamicKF()); lifts.enablePDControl(false);
                return false;
            }
            return true;
        }
    }

    public class SetDepositServosAction implements Action {
        private final double[] targetPositions;
        private boolean initialized = false;
        private static final double SERVO_ACTION_DURATION = 0.6;
        private ElapsedTime timer = new ElapsedTime();
        public SetDepositServosAction(double[] positions) { this.targetPositions = positions; }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                deposit.setServoPosition(1, targetPositions[0]);
                deposit.setServoPosition(2, targetPositions[1]);
                deposit.setServoPosition(3, targetPositions[2]);
                deposit.setServoPosition(4, targetPositions[3]);
                timer.reset(); initialized = true;
                packet.put("DepositServosAction", "Claw to " + targetPositions[3]);
            }
            if (timer.seconds() > SERVO_ACTION_DURATION) return false;
            return true;
        }
    }


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // --- Define Key Poses (Tune these for your robot and alliance color!) ---
        Pose2d startPose = new Pose2d(12, -60, Math.toRadians(90)); // Example
        Pose2d scorePose_preload = new Pose2d(48, -36, Math.toRadians(180));
        Pose2d sample1Pose = new Pose2d(12, -12, Math.toRadians(180));
        Pose2d sample2Pose = new Pose2d(-24, -12, Math.toRadians(180));
        Pose2d sample3Pose = new Pose2d(-48, -12, Math.toRadians(180));
        Pose2d scanPose = new Pose2d(36, -12, Math.toRadians(180));
        // Define absolute poses for scoring and parking
        Pose2d actualScoreDropPose = new Pose2d(50, -30, Math.toRadians(180)); // EXAMPLE: Tune this
        Pose2d parkingPoseYellowFound = new Pose2d(60, -12, Math.toRadians(180)); // EXAMPLE
        Pose2d parkingPoseNoYellow = new Pose2d(48, -60, Math.toRadians(180)); // EXAMPLE

        drive = new MecanumDrive(hardwareMap, startPose);

        double currentVoltage = 12.0;
        controlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        try { currentVoltage = controlHubVoltageSensor.getVoltage(); }
        catch (Exception e) { telemetry.addLine("No 'Control Hub' voltage sensor? Using 12V default for Lifts."); }

        lifts = new Lifts(hardwareMap, currentVoltage);
        intake = new Intake(hardwareMap); // Ensure PID constants in Intake are tuned
        deposit = new Deposit(hardwareMap);

        int roiWidth = 200; int roiHeight = 200;
        int roiX = (cameraWidth - roiWidth) / 2; int roiY = (cameraHeight - roiHeight) / 2;
        inspectionROI = new Rect(roiX, roiY, roiWidth, roiHeight);
        initializeVision();

        telemetry.addData("Status", "Initialized. Voltage: " + String.format("%.2f", currentVoltage) + ". Waiting for Start...");
        telemetry.addData("Start Pose", startPose);
        telemetry.update();

        // --- Define All Fixed TrajectoryActionBuilders and their Actions upfront ---
        telemetry.addLine("Defining pre-scan trajectories...");
        TrajectoryActionBuilder builderToPreloadScore = drive.actionBuilder(startPose)
                .splineToLinearHeading(scorePose_preload, Math.toRadians(0)) // End tangent for spline
                .endTrajectory();
        Action driveToPreloadScore = builderToPreloadScore.build();

        TrajectoryActionBuilder builderToSample1 = builderToPreloadScore.endTrajectory().fresh()
                .splineToLinearHeading(sample1Pose,Math.PI/2) //lineToLinearHeading uses heading from sample1Pose
                .endTrajectory();
        Action driveToSample1 = builderToSample1.build();

        TrajectoryActionBuilder builderToSample2 = builderToSample1.endTrajectory().fresh()
                .splineToLinearHeading(sample2Pose,Math.PI/2)
                .endTrajectory();
        Action driveToSample2 = builderToSample2.build();

        TrajectoryActionBuilder builderToSample3 = builderToSample2.endTrajectory().fresh()
                .splineToLinearHeading(sample3Pose,Math.PI/2)
                .endTrajectory();
        Action driveToSample3 = builderToSample3.build();

        TrajectoryActionBuilder builderToScanPose = builderToSample3.endTrajectory().fresh()
                .splineToLinearHeading(scanPose,Math.PI/2)
                .endTrajectory();
        Action driveToScan = builderToScanPose.build();
        telemetry.addLine("Pre-scan trajectories defined.");


        Action liftUpForPreload = new MoveLiftPDFLAction(Constants.liftExtensionValues[3]); // Assuming index 3 is preload score height
        Action depositAngleForPreload = new SetDepositServosAction(Constants.outtakeSampleDrop);
        Action depositResetAfterPreload = new SetDepositServosAction(Constants.outtakeSampleReset);
        Action liftRetractAfterPreload = new MoveLiftPDFLAction(Constants.liftExtensionValues[0]); // Assuming index 0 is retract/transfer
        Action scanAction = new Action() {
            @Override public boolean run(@NonNull TelemetryPacket packet) {
                performScan(); return false;
            }
        };

        Action preScanSequence = new SequentialAction(
                new ParallelAction(
                        driveToPreloadScore,
                        liftUpForPreload,
                        depositAngleForPreload
                ),
                new SleepAction(0.5),
                depositResetAfterPreload,
                liftRetractAfterPreload,
                driveToSample1,
                driveToSample2,
                driveToSample3,
                driveToScan,
                scanAction
        );

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Status", isWebcamStreaming ? "Webcam Streaming. Ready." : "Waiting for webcam...");
            telemetry.update();
            sleep(50);
        }

        if (isStopRequested()) { cleanup(); return; }
        if (!isWebcamStreaming) {
            telemetry.addLine("Webcam did not start. Aborting.");
            telemetry.update(); sleep(2000); cleanup(); return;
        }

        if (opModeIsActive()) {
            telemetry.addLine("Executing pre-scan sequence...");
            telemetry.update();
            Actions.runBlocking(preScanSequence);
        }
        if (isStopRequested()) { cleanup(); return; }

        List<Action> postScanActionList = new ArrayList<>();
        TrajectoryActionBuilder postScanPathBuilder = builderToScanPose.fresh(); // Builder is fresh, at scanPose
        boolean yellowFoundAndActed = false;

        if (detectedPosition != DetectionResult.NONE) {
            telemetry.addLine("Yellow found at scan pose!");
            postScanPathBuilder = addFullGrabAndScoreActionsToList(postScanActionList, postScanPathBuilder, actualScoreDropPose);
            yellowFoundAndActed = true;
        } else {
            telemetry.addLine("No yellow at initial scan pose. Starting incremental intake scan...");
            // IMPORTANT: Define these constants in your Constants.java
            // e.g., public static final int INTAKE_EXTENSION_SCAN_START_TICKS = 100;
            //       public static final int INTAKE_EXTENSION_SCAN_INCREMENT_TICKS = 150;
            //       public static final int INTAKE_EXTENSION_MAX_SCAN_TICKS = 700;
            // Using intakeExtensionValues as per user's code structure.
            // Ensure these indices are correct for your Constants.
            int currentIntakeScanTarget = Constants.intakeExtensionValues[0] + 200; // Example starting point relative to retracted
            int scanIncrement = Constants.intakeExtensionValues[2]; // Assuming index 2 is increment
            int maxExtension = Constants.intakeExtensionValues[1]; // Assuming index 1 is max extend

            for (int attempt = 0; attempt < MAX_SCAN_ATTEMPTS && !yellowFoundAndActed && opModeIsActive(); attempt++) {
                if (isStopRequested()) break;
                telemetry.addData("Incremental Intake Scan Attempt", (attempt + 1));
                telemetry.addData("Intake Target Ticks", currentIntakeScanTarget);

                Action extendIntakeForThisScan = new MoveIntakeExtensionPIDAction(currentIntakeScanTarget);
                // Drivetrain does not move during intake scan; postScanPathBuilder remains at scanPose
                Actions.runBlocking(new SequentialAction(extendIntakeForThisScan, scanAction));

                if (detectedPosition != DetectionResult.NONE) {
                    telemetry.addLine("Yellow found on incremental intake scan!");
                    postScanPathBuilder = addFullGrabAndScoreActionsToList(postScanActionList, postScanPathBuilder, actualScoreDropPose);
                    yellowFoundAndActed = true;
                    break;
                }
                currentIntakeScanTarget += scanIncrement;
                if (currentIntakeScanTarget > maxExtension) {
                    telemetry.addLine("Max intake scan extension reached.");
                    break;
                }
            }
        }

        TrajectoryActionBuilder parkBuilder;
        if (!yellowFoundAndActed) {
            telemetry.addLine("No yellow found. Parking.");
            // postScanPathBuilder is fresh, starting at scanPose (if no yellow found and no incremental drive)
            // or at the end of the scoring sequence's drive if yellow was found.
            // For "no yellow found", it's still at scanPose.
            parkBuilder = postScanPathBuilder.splineToLinearHeading(parkingPoseNoYellow,Math.PI/2).endTrajectory();
        } else {
            telemetry.addLine("Actions complete. Parking.");
            // postScanPathBuilder was returned by addFullGrabAndScoreActionsToList,
            // fresh from the end of the scoring drive.
            parkBuilder = postScanPathBuilder.splineToLinearHeading(parkingPoseYellowFound,Math.PI/2).endTrajectory();
        }
        postScanActionList.add(parkBuilder.build());

        if (!postScanActionList.isEmpty() && opModeIsActive()) {
            telemetry.addLine("Executing post-scan actions...");
            Actions.runBlocking(new SequentialAction(postScanActionList));
        }

        telemetry.addData("Status", "Autonomous Finished");
        telemetry.update();
        sleep(1000);
        cleanup();
    }

    private TrajectoryActionBuilder addFullGrabAndScoreActionsToList(List<Action> sequence, TrajectoryActionBuilder builderToStartDriveFrom, Pose2d scoreDropPose) {
        TrajectoryActionBuilder currentFuncBuilder = builderToStartDriveFrom; // This builder is already "fresh"

        sequence.add(new SetIntakeServosAction(Constants.intakeActive));
        sequence.add(new MoveIntakeExtensionPIDAction(Constants.intakeExtensionValues[1])); // EXTEND
        sequence.add(new SleepAction(0.2));
        sequence.add(new SetIntakeServosAction(Constants.intakeSampleReset)); // Secure
        sequence.add(new MoveIntakeExtensionPIDAction(Constants.intakeExtensionValues[0])); // RETRACT
        sequence.add(new MoveLiftPDFLAction(Constants.liftExtensionValues[0])); // LIFT_TRANSFER_HEIGHT
        sequence.add(new SetDepositServosAction(Constants.outtakeSampleReset));
        sequence.add(new SleepAction(0.75));
        sequence.add(new MoveLiftPDFLAction(Constants.liftExtensionValues[3])); // LIFT_SCORE_HEIGHT

        // Drive to the absolute scoring position
        currentFuncBuilder = currentFuncBuilder.splineToLinearHeading(scoreDropPose,Math.PI/2) // Drive to the predefined absolute score pose
                .endTrajectory();
        sequence.add(currentFuncBuilder.build());
        currentFuncBuilder = currentFuncBuilder.fresh(); // Freshen for any potential next step

        sequence.add(new SetDepositServosAction(Constants.outtakeSampleDrop));
        sequence.add(new SleepAction(1.0));
        sequence.add(new SetDepositServosAction(Constants.outtakeSampleReset));
        sequence.add(new MoveLiftPDFLAction((int)Constants.outtakeSampleReset[4]));

        return currentFuncBuilder;
    }

    private void initializeVision() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        visionPipeline = new ContrastAndROI();
        visionPipeline.setInspectionROI(inspectionROI);
        webcam.setPipeline(visionPipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override public void onOpened() { webcam.startStreaming(cameraWidth, cameraHeight, OpenCvCameraRotation.UPRIGHT); isWebcamStreaming = true; }
            @Override public void onError(int errorCode) { isWebcamStreaming = false; telemetry.addData("Error", "Webcam Error: " + errorCode); }
        });
    }

    private void performScan() {
        if (!isWebcamStreaming || !opModeIsActive()) return;
        telemetry.addLine("Performing scan...");
        telemetry.update();
        scanTimer.reset();
        detectedPosition = DetectionResult.NONE;
        while (opModeIsActive() && !isStopRequested() && scanTimer.seconds() < SCAN_DURATION_SEC) {
            List<Rect> currentDetections = visionPipeline.getDetectedRectangles();
            if (!currentDetections.isEmpty()) {
                Rect firstObject = currentDetections.get(0);
                double objectCenterXInFullFrame = firstObject.x + firstObject.width / 2.0;
                double roiThirdWidth = inspectionROI.width / 3.0;
                double roiLeftBoundary = inspectionROI.x + roiThirdWidth;
                double roiCenterBoundary = inspectionROI.x + (2.0 * roiThirdWidth);
                if (objectCenterXInFullFrame < roiLeftBoundary) detectedPosition = DetectionResult.LEFT;
                else if (objectCenterXInFullFrame < roiCenterBoundary) detectedPosition = DetectionResult.CENTER;
                else if (objectCenterXInFullFrame < inspectionROI.x + inspectionROI.width) detectedPosition = DetectionResult.RIGHT;
                else detectedPosition = DetectionResult.NONE;
            } else {
                detectedPosition = DetectionResult.NONE;
            }
            sleep(50);
        }
        telemetry.addData("Scan complete. Result", detectedPosition);
        telemetry.update();
    }

    private void cleanup() {
        if (webcam != null) {
            if (isWebcamStreaming) webcam.stopStreaming();
            webcam.closeCameraDevice();
            isWebcamStreaming = false;
        }
        if (visionPipeline != null) {
            try { visionPipeline.getClass().getMethod("cleanup").invoke(visionPipeline); }
            catch (Exception e) { /* Ignored */ }
        }
        if (lifts != null) {
            lifts.enablePDControl(false);
            lifts.stopSlides();
        }
        if (intake != null) {
            intake.setMotorPower(0);
        }
        telemetry.addLine("Cleanup complete.");
        telemetry.update();
    }
}
