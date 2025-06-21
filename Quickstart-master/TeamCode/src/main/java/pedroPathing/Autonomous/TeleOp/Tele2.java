package pedroPathing.Autonomous.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import pedroPathing.Autonomous.ParellelExp;
import pedroPathing.Robot.Commands.DepositResetCommand;
import pedroPathing.Robot.Commands.IntakeExtensionControlCommand;
import pedroPathing.Robot.Commands.IntakeGrabCommand;
import pedroPathing.Robot.Commands.IntakeHorizontalSpinCommand;
import pedroPathing.Robot.Commands.PDFLCommand;
import pedroPathing.Robot.Constants;
import pedroPathing.Robot.Sensing.LimelightBaseSystem;
import pedroPathing.Robot.Sensing.Timer;
import pedroPathing.Robot.Systems.Deposit;
import pedroPathing.Robot.Systems.Intake;
import pedroPathing.Robot.Systems.Lifts;


@TeleOp(name = "TeleOp - Main Att 2")
public class Tele2  extends LinearOpMode {
    //hardware
    public DcMotorEx leftFront, leftBack, rightFront, rightBack;
    public Lifts lifts;
    public Intake intake;
    public Deposit deposit;
    public VoltageSensor voltageSensor;
    public LimelightBaseSystem limelight;
    public Timer voltageSensorTimer;

    //Variables
    public boolean lastGamepad1LeftBumperState = false;
    public boolean lastdPadDownState = false;
    public boolean lastdPadleftDownState = false;
    public boolean lastBButState = false;
    public boolean lastAButState = false;
    public boolean voltageSpiked = false;
    int currentAState = 1;
    int currentStateB = 1;
    int currentStateDpad = 1;
    int currentStateDpadLeft = 1;
    public double speed = 1.0;
    public boolean autograb = false;
    public boolean intakeready = false;
    private double initialVoltage;
    LiftState currentState = LiftState.INTAKE;

    // Define a clear, readable set of states for the lift
    public enum LiftState {
        INTAKE,          // The lift is down, ready for intake
        SCORING_HIGH,    // The lift is raised to the high scoring position
        DEPOSIT_READY    // The deposit has been reset, ready to move again
    }



    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = FtcDashboard.getInstance().getTelemetry();

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        limelight = new LimelightBaseSystem(hardwareMap, "limelight");
        limelight.startLimelight();

        initialVoltage = voltageSensor.getVoltage();
        voltageSensorTimer = new Timer();

        lifts = new Lifts(hardwareMap, initialVoltage);
        intake = new Intake(hardwareMap);
        deposit = new Deposit(hardwareMap);
        CommandScheduler.getInstance().reset();


        deposit.setServoPosition(1, Constants.outtakeSampleReset[0]);
        deposit.setServoPosition(2, Constants.outtakeSampleReset[1]);
        deposit.setServoPosition(3, Constants.outtakeSampleReset[2]);
        deposit.setServoPosition(4,Constants.outtakeSampleReset[3]);

        intake.setServoPosition(1, 0.8);
        intake.setServoPosition(2, 0.2);
        intake.setServoPosition(3,0.5);
        intake.setServoPosition(4,0.5);
        intake.setServoPosition(6, Constants.intakeActive[4]);

        waitForStart();
        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();
            telemetry.addData("voltage", voltageSensor.getVoltage());
            telemetry.addData("reference voltage", initialVoltage);


            // In your class, declare the state variable using the new enum
            // Read the current button state once per loop
            boolean currentGamepad1LeftBumperState = gamepad1.left_bumper;
            // --- Edge Detector: This part is good, it runs the code only on the button press ---
            if (currentGamepad1LeftBumperState && !lastGamepad1LeftBumperState) {
                // Use a switch statement for cleaner state management
                switch (currentState) {
                    case INTAKE:
                        // From INTAKE, move to the SCORING_HIGH position
                        new PDFLCommand(lifts, 1475).schedule();
                        currentState = LiftState.DEPOSIT_READY;
                        telemetry.addLine("Lift State -> SCORING_HIGH");
                        break;

                    case DEPOSIT_READY:
                        // From DEPOSIT_READY, go back to the INTAKE position
                        new DepositResetCommand(deposit).schedule();
                        new PDFLCommand(lifts, 300).schedule();
                        currentState = LiftState.INTAKE;
                        telemetry.addLine("Lift State -> INTAKE");
                        break;
                }
            }
            // Update the last state for the next loop iteration
            lastGamepad1LeftBumperState = currentGamepad1LeftBumperState;
            telemetry.update();
            dpdown();
            dpdLeft();


            if (gamepad1.dpad_up) {
                new PDFLCommand(lifts, 1450).schedule();
            }


            if (gamepad1.right_bumper) {
                deposit.setServoPosition(4,Constants.outtakeSampleReset[3]);
                //new PDFLCommand(lifts, 75).schedule();
            }
            //auto extends winch once end game starts

           // if (gamepad1.a) {
             //   speed = 0.6;
               // new IntakeExtensionControlCommand(intake, Constants.intakeExtensionValues[1]).schedule();
                //intakeready = true;
            //}

            aButton();

            if(intakeready){
                if(intake.getHorizontalExtensionVelocity() <= 10) {
                    intake.setServoPosition(1, 0.80);
                    intake.setServoPosition(2, 0.2);
                    intake.setServoPosition(3, Constants.intakeActive[2]);
                    intake.setServoPosition(4, Constants.intakeActive[3]);
                    intake.setServoPosition(6, Constants.intakeActive[4]); // Corrected index if for claw
                    intakeready = false;
                }
            }
            bButton();


            //if (gamepad1.b) {
              //  speed = 1.0;
                //new SequentialCommandGroup(new IntakeGrabCommand(intake), new WaitCommand(200), new InstantCommand(() -> intake.setServoPosition(6, 0.44)), new IntakeExtensionControlCommand(intake, Constants.intakeExtensionValues[0])).schedule();
           // }


            if(autograb){
                if(lifts.getTopMotorData()[0] < 15) {
                    deposit.setServoPosition(4, Constants.outtakeSampleDrop[3]);
                    sleep(150);
                    intake.setServoPosition(6, Constants.intakeActive[4]);
                    sleep(150);
                    deposit.setServoPosition(1, Constants.outtakeSampleDrop[0] - 0.05);
                    deposit.setServoPosition(2, Constants.outtakeSampleDrop[1] + 0.05);
                    deposit.setServoPosition(3, Constants.outtakeSampleDrop[2]);
                    autograb = false;
                    currentStateB = 1;
                }
            }

            if (gamepad1.x) {
                new PDFLCommand(lifts,0).schedule();
                autograb = true;
            }

            //if (gamepad1.dpad_down) {
            //    new IntakeHorizontalSpinCommand(intake).schedule();

            //}

            if (gamepad1.y) {
                new DepositResetCommand(deposit).schedule();

            }


            double rx = gamepad1.right_stick_x;
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            leftFront.setPower((y + rx + x) / denominator * speed);
            leftBack.setPower((y - x + rx) / denominator * speed);
            rightFront.setPower((y - rx - x) / denominator * speed);
            rightBack.setPower((y - rx + x) / denominator * speed);
        }


    }

        // Update the last known state of the bumper for the next loop iteration}

    public void dpdown(){
        boolean currentdPadGameState = gamepad1.dpad_down;
        if (currentdPadGameState && !lastdPadDownState) {
            if(currentStateDpad == 1) {
                new IntakeHorizontalSpinCommand(intake).schedule();
                currentStateDpad = 2;
            } else {
                new InstantCommand(() -> {
                    intake.setServoPosition(3, 0.5);
                    intake.setServoPosition(4, 0.5);
                }).schedule();
                currentStateDpad = 1;
            }
        }
        lastdPadDownState = currentdPadGameState;
    }

    public void dpdLeft(){
        boolean currentdPadLeftGameState = gamepad1.dpad_left;
        if (currentdPadLeftGameState && !lastdPadleftDownState) {
            if(currentStateDpadLeft == 1) {
                new PDFLCommand(lifts, 700).schedule();
                currentStateDpadLeft = 2;
            } else {
                new PDFLCommand(lifts, 50).schedule();
                currentStateDpadLeft = 1;
            }
        }
        lastdPadleftDownState = currentdPadLeftGameState;
    }

    public void bButton(){
        boolean curentbButState = gamepad1.b;
        if(curentbButState && !lastBButState){
            if(currentStateB ==1){
                speed = 1.0;
                initialVoltage = voltageSensor.getVoltage();
                new IntakeGrabCommand(intake).schedule();
                currentStateB = 2;
            } else{
                speed = 0.6;
                intakeready = true;
                currentStateB = 1;
            }
        }
        lastBButState = curentbButState;
    }

    public void aButton(){
        boolean currentaButState = gamepad1.a;
        if(currentaButState && !lastAButState){
            if(currentAState == 1){
                speed = 0.6;
                new IntakeExtensionControlCommand(intake, Constants.intakeExtensionValues[1]).schedule();
                intakeready = true;
                currentAState = 2;
            } else{
                speed = 1.0;
                new IntakeExtensionControlCommand(intake, 0).schedule();
                intake.setServoPosition(6,0.44);
                intakeready = false;
                currentAState = 1;
            }
        }
        lastAButState = currentaButState;

    }

}
