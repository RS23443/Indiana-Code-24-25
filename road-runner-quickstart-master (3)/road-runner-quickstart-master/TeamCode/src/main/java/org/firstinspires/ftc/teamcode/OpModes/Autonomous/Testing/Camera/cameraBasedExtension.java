package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Testing.Camera; // Or any suitable package for your tests

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp; // Or @Autonomous if you prefer for testing
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket; // Required for Action's run method

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot.Camera.Pipelines.ContrastAndROI;
import org.firstinspires.ftc.teamcode.Robot.Constants; // For intake extension targets
import org.firstinspires.ftc.teamcode.Robot.Systems.Intake;

import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

@TeleOp(name = "Test: Intake Extension Scan", group = "Test")
// @Disabled
public class cameraBasedExtension extends LinearOpMode {

    OpenCvCamera webcam;
    ContrastAndROI visionPipeline;
    DetectionResult detectedPosition = DetectionResult.NONE;
    int cameraWidth = 640; int cameraHeight = 480; // Match your main auto
    Rect inspectionROI; // Define this based on your camera's view when intake is extended

    private volatile boolean isWebcamStreaming = false;
    private static final double SCAN_DURATION_SEC = 2.0; // Allow a bit more time for testing
    private ElapsedTime scanTimer = new ElapsedTime();

    public Intake intake;

    // Define scan positions for intake extension (in ticks)
    // IMPORTANT: You MUST define these constants in your Constants.java or set them here.
    // These are just placeholders.
    // Example: Constants.INTAKE_EXTENSION_SCAN_POS_1, Constants.INTAKE_EXTENSION_SCAN_POS_2, etc.
    private int[] intakeScanPositions;


    public enum DetectionResult { LEFT, CENTER, RIGHT, NONE }

    // --- Action Class for Intake Extension (copied from main auto for self-containment) ---
    public class MoveIntakeExtensionPIDAction implements Action {
        private final int targetTicks;
        private boolean initialized = false;
        private ElapsedTime timeoutTimer = new ElapsedTime();
        private static final double EXTENSION_TIMEOUT_SEC = 3.0; // Slightly longer for testing
        private static final int POSITION_TOLERANCE = 20;

        public MoveIntakeExtensionPIDAction(int targetTicks) { this.targetTicks = targetTicks; }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!opModeIsActive()) { intake.setMotorPower(0); return false;}
            if (!initialized) {
                telemetry.addData("IntakeExtPIDAction: Moving to", targetTicks);
                telemetry.update(); // Show target immediately
                // Ensure Intake motor is in RUN_WITHOUT_ENCODER mode from Intake subsystem constructor
                // Ensure PID constants in Intake.java are tuned!
                initialized = true;
                timeoutTimer.reset();
            }

            intake.setMotor(targetTicks); // This method in your Intake class calls PID compute and setPower

            int currentPos = intake.getExtensionPosition();
            boolean atTarget = Math.abs(currentPos - targetTicks) < POSITION_TOLERANCE;

            packet.put("IntakeExt Target", targetTicks);
            packet.put("IntakeExt CurrentPos", currentPos);
            packet.put("IntakeExt AtTarget", atTarget);
            telemetry.addData("IntakeExt Target", targetTicks);
            telemetry.addData("IntakeExt CurrentPos", currentPos);


            if (atTarget) {
                telemetry.addLine("IntakeExtPIDAction: Reached target.");
                // intake.setMotorPower(0); // Let PID hold or explicitly stop
                intake.setMotorPower(0.05 * Math.signum(targetTicks - currentPos));
                if(Math.abs(targetTicks - currentPos) < 5) intake.setMotorPower(0);
                telemetry.update();
                return false; // Action complete
            }
            if (timeoutTimer.seconds() > EXTENSION_TIMEOUT_SEC) {
                telemetry.addLine("IntakeExtPIDAction: Timed out.");
                intake.setMotorPower(0);
                telemetry.update();
                return false; // Action complete (timeout)
            }
            telemetry.update();
            return true; // Still running
        }
    }


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initializing Test OpMode...");
        telemetry.update();

        intake = new Intake(hardwareMap); // Ensure PID constants in Intake are tuned!

        // Define ROI for vision - this should be where you expect to see the object
        // relative to the camera when the intake is in various extended positions.
        // For testing, a fairly central and generous ROI might be good.
        int roiWidth = 200; int roiHeight = 200;
        int roiX = (cameraWidth - roiWidth) / 2; int roiY = (cameraHeight - roiHeight) / 2;
        inspectionROI = new Rect(roiX, roiY, roiWidth, roiHeight);
        initializeVision();

        // Define intake extension positions for testing
        // IMPORTANT: Replace these with your actual Constants or tuned values!
        // These values are in motor ticks.
        intakeScanPositions = new int[]{
                Constants.intakeExtensionValues[0]+200, // e.g., 100
                Constants.intakeExtensionValues[0]+200 + Constants.intakeExtensionValues[2], // e.g., 250
                Constants.intakeExtensionValues[0]+200 + (2 * Constants.intakeExtensionValues[2]) // e.g., 400
                // Add more positions if needed, up to Constants.INTAKE_EXTENSION_MAX_SCAN_TICKS
        };


        telemetry.addData("Status", "Initialized. Waiting for Start to begin test cycle.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) { cleanup(); return; }

        // Test cycle
        for (int i = 0; i < intakeScanPositions.length && opModeIsActive(); i++) {
            int currentIntakeTarget = intakeScanPositions[i];
            telemetry.addData("Test Cycle", "Attempt " + (i + 1) + "/" + intakeScanPositions.length);
            telemetry.addData("Moving Intake Extension to", currentIntakeTarget);
            telemetry.update();

            Action moveIntake = new MoveIntakeExtensionPIDAction(currentIntakeTarget);
            Action scanAtPosition = new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    performScan(); // This is blocking for SCAN_DURATION_SEC
                    packet.put("Scan Result", detectedPosition.toString());
                    return false; // Action is done after scan
                }
            };

            // Run the sequence: move intake, then scan
            Actions.runBlocking(new SequentialAction(moveIntake, scanAtPosition));

            telemetry.addData("Scan Result at " + currentIntakeTarget + " ticks", detectedPosition);
            telemetry.addLine("------------------------------------");
            telemetry.update();

            if (isStopRequested()) break;

            // Pause before next test iteration, or wait for a button press
            sleep(2000); // Pause for 2 seconds to observe telemetry
            // Or, for interactive testing:
            // telemetry.addLine("Press (A) to continue to next position...");
            // telemetry.update();
            // while(opModeIsActive() && !gamepad1.a && !isStopRequested()) { sleep(20); }
            // if (isStopRequested()) break;
            // while(opModeIsActive() && gamepad1.a && !isStopRequested()) { sleep(20); } // Debounce
        }

        telemetry.addLine("Test cycle complete.");
        telemetry.update();
        sleep(5000); // Keep final telemetry visible

        cleanup();
    }

    private void initializeVision() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId); // Ensure "Webcam 1" is correct
        visionPipeline = new ContrastAndROI(); // Make sure this path is correct
        visionPipeline.setInspectionROI(inspectionROI);
        webcam.setPipeline(visionPipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override public void onOpened() {
                webcam.startStreaming(cameraWidth, cameraHeight, OpenCvCameraRotation.UPRIGHT);
                isWebcamStreaming = true;
                telemetry.addLine("Webcam streaming started for test.");
                telemetry.update();
            }
            @Override public void onError(int errorCode) {
                isWebcamStreaming = false;
                telemetry.addData("Error", "Webcam Error for test: " + errorCode);
                telemetry.update();
            }
        });
    }

    // Copied performScan method from main auto
    private void performScan() {
        if (!isWebcamStreaming || !opModeIsActive()) {
            detectedPosition = DetectionResult.NONE; // Ensure it's reset if not scanning
            telemetry.addLine("Scan skipped: Webcam not streaming or OpMode inactive.");
            telemetry.update();
            return;
        }
        telemetry.addLine("Performing scan...");
        telemetry.update(); // Show "Performing scan..."
        sleep(100); // Brief pause to ensure telemetry updates

        scanTimer.reset();
        detectedPosition = DetectionResult.NONE;
        DetectionResult frameDetection = DetectionResult.NONE; // Detection within a single frame

        while (opModeIsActive() && !isStopRequested() && scanTimer.seconds() < SCAN_DURATION_SEC) {
            List<Rect> currentDetections = visionPipeline.getDetectedRectangles();
            if (!currentDetections.isEmpty()) {
                Rect firstObject = currentDetections.get(0);
                double objectCenterXInFullFrame = firstObject.x + firstObject.width / 2.0;
                double roiThirdWidth = inspectionROI.width / 3.0;
                double roiLeftBoundary = inspectionROI.x + roiThirdWidth;
                double roiCenterBoundary = inspectionROI.x + (2.0 * roiThirdWidth);

                if (objectCenterXInFullFrame < roiLeftBoundary) frameDetection = DetectionResult.LEFT;
                else if (objectCenterXInFullFrame < roiCenterBoundary) frameDetection = DetectionResult.CENTER;
                else if (objectCenterXInFullFrame < inspectionROI.x + inspectionROI.width) frameDetection = DetectionResult.RIGHT;
                else frameDetection = DetectionResult.NONE;
            } else {
                frameDetection = DetectionResult.NONE;
            }
            // For testing, you might want to see what each frame detects
            // Or, implement logic to make `detectedPosition` more stable (e.g., majority vote over time)
            // For now, the last valid detection in the scan period will be used.
            if (frameDetection != DetectionResult.NONE) {
                detectedPosition = frameDetection;
            }

            telemetry.addData("Scanning...", String.format("%.1fs / %.1fs", scanTimer.seconds(), SCAN_DURATION_SEC));
            telemetry.addData("Frame Detection", frameDetection);
            telemetry.addData("Overall Scan Detection", detectedPosition);
            telemetry.update();
            sleep(50); // Allow pipeline to process, yield CPU
        }
        telemetry.addData("Scan complete. Final Result", detectedPosition);
        telemetry.update();
    }

    private void cleanup() {
        telemetry.addLine("Test OpMode cleaning up...");
        if (webcam != null) {
            if (isWebcamStreaming) webcam.stopStreaming();
            webcam.closeCameraDevice();
            isWebcamStreaming = false;
            telemetry.addLine("Webcam closed.");
        }
        if (visionPipeline != null && visionPipeline.getClass().getMethods() != null) { // Basic check
            try {
                // Check if cleanup method exists before calling, to prevent errors if it was removed
                java.lang.reflect.Method cleanupMethod = visionPipeline.getClass().getMethod("cleanup");
                if (cleanupMethod != null) {
                    cleanupMethod.invoke(visionPipeline);
                    telemetry.addLine("Vision pipeline cleaned.");
                }
            } catch (NoSuchMethodException e) {
                // No 'cleanup' method in pipeline, that's fine.
            } catch (Exception e) {
                telemetry.addData("Error", "Exception cleaning vision pipeline: " + e.getMessage());
            }
        }
        if (intake != null) {
            intake.setMotorPower(0); // Stop intake extension motor
            telemetry.addLine("Intake motor stopped.");
        }
        telemetry.addLine("Cleanup complete.");
        telemetry.update();
    }
}
