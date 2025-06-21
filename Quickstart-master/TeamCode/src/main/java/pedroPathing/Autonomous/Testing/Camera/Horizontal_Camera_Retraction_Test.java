package pedroPathing.Autonomous.Testing.Camera; // Adjust this package

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

import pedroPathing.Robot.Camera.Pipelines.ContrastAndROI;
import pedroPathing.Robot.Commands.Intake45Degrees;
import pedroPathing.Robot.Commands.IntakeExtensionControlCommand;
import pedroPathing.Robot.Commands.IntakeGrabCommand;
import pedroPathing.Robot.Constants;
import pedroPathing.Robot.Systems.Intake;

@TeleOp
// @Disabled
public class Horizontal_Camera_Retraction_Test  extends LinearOpMode {

    OpenCvCamera webcam;
    ContrastAndROI visionPipeline;
    DetectionResult detectedPosition = DetectionResult.NONE;

    int cameraWidth = 1280;
    int cameraHeight = 720;
    Rect inspectionROI;

    private volatile boolean isWebcamStreaming = false;
    private static final double SCAN_DURATION_SEC = 0.5;
    private ElapsedTime scanTimer = new ElapsedTime();

    public Intake intake;

    public enum DetectionResult { LEFT, CENTER, RIGHT, NONE }


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing Test OpMode (FTCLib Cmd)...");
        telemetry.update();

        intake = new Intake(hardwareMap);
        CommandScheduler.getInstance().reset();

        int roiWidth = cameraWidth;
        int roiHeight = cameraHeight;
        int roiX = 0;
        int roiY = 0;
        inspectionROI = new Rect(roiX, roiY, roiWidth, roiHeight);

        initializeVision();

        telemetry.addData("Status", "Initialized. Waiting for Start...");
        telemetry.addData("Pipeline ROI", "x: %d, y: %d, w: %d, h: %d",
                inspectionROI.x, inspectionROI.y, inspectionROI.width, inspectionROI.height);
        telemetry.update();
        intake.setServoPosition(6,0.7); // claw should close towards 0.0
        intake.setServoPosition(1,0.6);
        intake.setServoPosition(2,0.4);
        sleep(200);
        intake.setServoPosition(3,0.25);
        intake.setServoPosition(4,0.75);
        waitForStart();

        if (isStopRequested()) { cleanup(); return; }
        if (!isWebcamStreaming) {
            telemetry.addLine("Webcam did not start. Aborting test.");
            telemetry.update();
            sleep(3000);
            cleanup();
            return;
        }

        // --- Constants for Scanning Logic ---
        // Ensure these Constants.intakeExtensionValues indices are correct for your setup
        final int INITIAL_EXTENSION_FOR_FIRST_SCAN_TICKS = 100; // Explicit initial extension
        final int INTAKE_INCREMENT_TICKS = Constants.intakeExtensionValues[2];
        final int MAX_INTAKE_EXTENSION_TICKS = Constants.intakeExtensionValues[1];
        final int RETRACT_ON_DETECTION_AMOUNT_TICKS = Constants.intakeExtensionValues[3];
        final double VELOCITY_ZERO_THRESHOLD = Constants.intakeExtensionValues[4];

        boolean yellowFound = false;

        // --- 1. Initial Intake Extension to 100 Ticks ---
        telemetry.addData("Phase 1", "Extending intake to " + INITIAL_EXTENSION_FOR_FIRST_SCAN_TICKS + " for initial scan...");
        telemetry.update();

        IntakeExtensionControlCommand initialMoveCmd = new IntakeExtensionControlCommand(intake, INITIAL_EXTENSION_FOR_FIRST_SCAN_TICKS);
        initialMoveCmd.schedule();
        if(intake.getHorizontalExtensionVelocity() == 0){
            intake.setServoPosition(1,0.9);
            intake.setServoPosition(2,0.1);
            sleep(200);
            intake.setServoPosition(3,0.5);
            intake.setServoPosition(4,0.5);
            sleep(200);
            intake.setServoPosition(6,0.48);
            sleep(200);
            intake.setServoPosition(1,0.6);
            intake.setServoPosition(2,0.4);
            sleep(200);
            intake.setServoPosition(3,0.25);
            intake.setServoPosition(4,0.75);
        }
        IntakeExtensionControlCommand backToHome = new IntakeExtensionControlCommand(intake, 0);
        backToHome.schedule();
        ElapsedTime cmdTimeout = new ElapsedTime();
        while(opModeIsActive() && !isStopRequested() && !initialMoveCmd.isFinished() && cmdTimeout.seconds() < 3.0) {
            CommandScheduler.getInstance().run();
            telemetry.addData("Initial Intake Moving to", INITIAL_EXTENSION_FOR_FIRST_SCAN_TICKS);
            telemetry.addData("Initial Intake Current Pos", intake.getExtensionPosition());
            telemetry.update();
            sleep(20);
        }
        if (!initialMoveCmd.isFinished()) {
            initialMoveCmd.cancel();
        }
        CommandScheduler.getInstance().run(); // Process end/cancel
        telemetry.addLine("Initial intake extension complete.");
        telemetry.update();

        if (isStopRequested()) { cleanup(); return; }

        // --- 2. Settle and Perform First Scan at 100 Ticks ---
        telemetry.addLine("Waiting for intake to settle for first scan...");
        telemetry.update();
        ElapsedTime velocitySettleTimer = new ElapsedTime();
        while(opModeIsActive() &&
                Math.abs(intake.getHorizontalExtensionVelocity()) > VELOCITY_ZERO_THRESHOLD &&
                velocitySettleTimer.seconds() < 1.0) {
            telemetry.addData("Current Intake Velocity", String.format("%.2f", intake.getHorizontalExtensionVelocity()));
            telemetry.addData("Waiting for Settle", String.format("%.2f s", velocitySettleTimer.seconds()));
            telemetry.update();
            sleep(20);
        }
        if (Math.abs(intake.getHorizontalExtensionVelocity()) > VELOCITY_ZERO_THRESHOLD) {
            telemetry.addLine(String.format("Warning: Intake motor velocity (%.2f) still high.", intake.getHorizontalExtensionVelocity()));
        } else {
            telemetry.addLine("Intake motor settled. Performing initial scan.");
        }
        telemetry.update();

        performScan(); // Perform the first scan at INITIAL_EXTENSION_FOR_FIRST_SCAN_TICKS

        telemetry.addData("Initial Scan Result at " + INITIAL_EXTENSION_FOR_FIRST_SCAN_TICKS + " ticks", detectedPosition);
        telemetry.update();

        if (detectedPosition != DetectionResult.NONE) {
            yellowFound = true;
            telemetry.addData("Yellow detected at initial position", intake.getExtensionPosition());
            int retractToTarget = Math.max(0, intake.getExtensionPosition() - RETRACT_ON_DETECTION_AMOUNT_TICKS);
            telemetry.addData("Retracting intake to", retractToTarget);
            telemetry.update();

            IntakeExtensionControlCommand retractCmd = new IntakeExtensionControlCommand(intake, retractToTarget);
            retractCmd.schedule();
            if(intake.getHorizontalExtensionVelocity() == 0){
                intake.setServoPosition(1,0.9);
                intake.setServoPosition(2,0.1);
                sleep(200);
                intake.setServoPosition(3,0.5);
                intake.setServoPosition(4,0.5);
                sleep(200);
                intake.setServoPosition(6,0.48);
                sleep(200);
                intake.setServoPosition(1,0.6);
                intake.setServoPosition(2,0.4);
                sleep(200);
                intake.setServoPosition(3,0.25);
                intake.setServoPosition(4,0.75);
            }
            IntakeExtensionControlCommand backToHomeInc = new IntakeExtensionControlCommand(intake, 0);
            backToHomeInc.schedule();
            cmdTimeout.reset();
            while(opModeIsActive() && !isStopRequested() && !retractCmd.isFinished() && cmdTimeout.seconds() < 3.0) {
                CommandScheduler.getInstance().run();
                telemetry.addData("Intake Retracting to", retractToTarget);
                telemetry.addData("Intake Current Pos", intake.getExtensionPosition());
                telemetry.update();
                sleep(20);
            }
            if (!retractCmd.isFinished()) { retractCmd.cancel(); }
            CommandScheduler.getInstance().run();
            telemetry.addLine("Intake retracted after initial detection.");
            telemetry.update();
            sleep(3000);
        } else {
            telemetry.addLine("No yellow detected at initial 100-tick scan. Starting incremental process.");
            telemetry.update();
            sleep(1000); // Pause before starting incremental
        }

        // --- 3. Incremental Intake Scan Loop (only if not found in step 2) ---
        if (opModeIsActive() && !isStopRequested() && !yellowFound) {
            int currentIntakeTarget = INITIAL_EXTENSION_FOR_FIRST_SCAN_TICKS + INTAKE_INCREMENT_TICKS; // Start from the next increment

            while (opModeIsActive() && currentIntakeTarget <= MAX_INTAKE_EXTENSION_TICKS && !yellowFound) {
                telemetry.addData("Incremental Cycle", "Moving intake to: " + currentIntakeTarget + " ticks");
                telemetry.update();

                IntakeExtensionControlCommand moveCmdLoop = new IntakeExtensionControlCommand(intake, currentIntakeTarget);
                moveCmdLoop.schedule();
                cmdTimeout.reset();
                while(opModeIsActive() && !isStopRequested() && !moveCmdLoop.isFinished() && cmdTimeout.seconds() < 3.0) {
                    CommandScheduler.getInstance().run();
                    telemetry.addData("Intake Moving to", currentIntakeTarget);
                    telemetry.addData("Intake Current Pos", intake.getExtensionPosition());
                    telemetry.update();
                    sleep(20);
                }
                if (!moveCmdLoop.isFinished()) { moveCmdLoop.cancel(); }
                CommandScheduler.getInstance().run();

                if (isStopRequested()) break;

                telemetry.addLine("Intake move command finished. Waiting for velocity to be near zero...");
                velocitySettleTimer.reset();
                while(opModeIsActive() &&
                        Math.abs(intake.getHorizontalExtensionVelocity()) > VELOCITY_ZERO_THRESHOLD &&
                        velocitySettleTimer.seconds() < 1.0) {
                    telemetry.addData("Current Intake Velocity", String.format("%.2f", intake.getHorizontalExtensionVelocity()));
                    telemetry.addData("Waiting for Settle", String.format("%.2f s", velocitySettleTimer.seconds()));
                    telemetry.update();
                    sleep(20);
                }
                if (Math.abs(intake.getHorizontalExtensionVelocity()) > VELOCITY_ZERO_THRESHOLD) {
                    telemetry.addLine(String.format("Warning: Intake velocity (%.2f) still high.", intake.getHorizontalExtensionVelocity()));
                } else {
                    telemetry.addLine("Intake motor settled. Performing scan.");
                }
                telemetry.update();

                performScan();

                telemetry.addData("Scan Result at " + currentIntakeTarget + " ticks", detectedPosition);
                telemetry.update();

                if (detectedPosition != DetectionResult.NONE) {
                    yellowFound = true;
                    telemetry.addData("Yellow detected at intake position", intake.getExtensionPosition());
                    int retractToTarget = Math.max(0, intake.getExtensionPosition() - RETRACT_ON_DETECTION_AMOUNT_TICKS);
                    telemetry.addData("Retracting intake to", retractToTarget);
                    telemetry.update();

                    IntakeExtensionControlCommand retractCmdLoop = new IntakeExtensionControlCommand(intake, retractToTarget);
                    retractCmdLoop.schedule();
                    new Intake45Degrees(intake).schedule();
                    new WaitCommand(200).schedule();
                    new IntakeGrabCommand(intake).schedule();                    cmdTimeout.reset();
                    while(opModeIsActive() && !isStopRequested() && !retractCmdLoop.isFinished() && cmdTimeout.seconds() < 3.0) {
                        CommandScheduler.getInstance().run();
                        telemetry.addData("Intake Retracting to", retractToTarget);
                        telemetry.addData("Intake Current Pos", intake.getExtensionPosition());
                        telemetry.update();
                        sleep(20);
                    }
                    if (!retractCmdLoop.isFinished()) { retractCmdLoop.cancel(); }
                    CommandScheduler.getInstance().run();

                    telemetry.addLine("Intake retracted. Detection cycle complete.");
                    telemetry.update();
                    sleep(3000);
                } else {
                    telemetry.addLine("No yellow detected at this position.");
                    telemetry.update();
                    if (currentIntakeTarget >= MAX_INTAKE_EXTENSION_TICKS) {
                        break;
                    }
                    currentIntakeTarget += INTAKE_INCREMENT_TICKS;
                    if (currentIntakeTarget > MAX_INTAKE_EXTENSION_TICKS) {
                        currentIntakeTarget = MAX_INTAKE_EXTENSION_TICKS;
                    }
                    if(!yellowFound) sleep(1000);
                }
            }
        }

        if (!yellowFound && opModeIsActive()) {
            telemetry.addLine("Full scan sequence complete. No yellow object found.");
            telemetry.update();
            sleep(3000);
        }

        telemetry.addLine("Test OpMode Finished.");
        telemetry.update();
        sleep(1000);

        cleanup();
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
            @Override public void onOpened() {
                webcam.startStreaming(cameraWidth, cameraHeight, OpenCvCameraRotation.UPRIGHT);
                isWebcamStreaming = true;
            }
            @Override public void onError(int errorCode) {
                isWebcamStreaming = false;
                telemetry.addData("Error", "Webcam Error: " + errorCode);
            }
        });
    }

    private void performScan() {
        if (!isWebcamStreaming || !opModeIsActive()) {
            detectedPosition = DetectionResult.NONE;
            telemetry.addLine("Scan skipped: Webcam not streaming or OpMode inactive.");
            telemetry.update();
            return;
        }
        telemetry.addLine("Performing scan...");
        telemetry.update();
        sleep(100);

        scanTimer.reset();
        detectedPosition = DetectionResult.NONE;
        DetectionResult frameDetection = DetectionResult.NONE;

        while (opModeIsActive() && !isStopRequested() && scanTimer.seconds() < SCAN_DURATION_SEC) {
            List<Rect> currentDetections = visionPipeline.getDetectedRectangles();
            if (!currentDetections.isEmpty()) {
                Rect firstObject = currentDetections.get(0);
                double objectCenterXInFullFrame = firstObject.x + firstObject.width / 2.0;
                double frameThirdWidth = (double)cameraWidth / 3.0;
                double leftBoundary = frameThirdWidth;
                double centerBoundary = 2.0 * frameThirdWidth;

                if (objectCenterXInFullFrame < leftBoundary) frameDetection = DetectionResult.LEFT;
                else if (objectCenterXInFullFrame < centerBoundary) frameDetection = DetectionResult.CENTER;
                else frameDetection = DetectionResult.RIGHT;
            } else {
                frameDetection = DetectionResult.NONE;
            }
            if (frameDetection != DetectionResult.NONE) {
                detectedPosition = frameDetection;
            }

            telemetry.addData("Scanning...", String.format("%.1fs / %.1fs", scanTimer.seconds(), SCAN_DURATION_SEC));
            telemetry.addData("Frame Detection", frameDetection);
            telemetry.addData("Overall Scan Detection", detectedPosition);
            telemetry.update();
            sleep(50);
        }
        telemetry.addData("Scan complete. Final Result for this position", detectedPosition);
        telemetry.update();
    }

    private void cleanup() {
        telemetry.addLine("Test OpMode cleaning up...");
        CommandScheduler.getInstance().reset();
        if (webcam != null) {
            if (isWebcamStreaming) webcam.stopStreaming();
            webcam.closeCameraDevice();
            isWebcamStreaming = false;
            telemetry.addLine("Webcam closed.");
        }
        if (visionPipeline != null) {
            try {
                java.lang.reflect.Method cleanupMethod = visionPipeline.getClass().getMethod("cleanup");
                cleanupMethod.invoke(visionPipeline);
                telemetry.addLine("Vision pipeline cleaned.");
            } catch (NoSuchMethodException e) { /* No 'cleanup' method */
            } catch (Exception e) { telemetry.addData("Error", "Exception cleaning vision pipeline: " + e.getMessage()); }
        }
        if (intake != null) {
            intake.setMotorPower(0);
            telemetry.addLine("Intake motor stopped.");
        }
        telemetry.addLine("Cleanup complete.");
        telemetry.update();
    }
}
