package org.firstinspires.ftc.teamcode.OpModes.Autonomous.TeleOp;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Robot.Commands.DepositDropPositionCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.DepositResetCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.IntakeExtensionControlCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.IntakeGrabCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.IntakeHorizontalSpinCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.PDFLCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.SampleSwitchCommand;
import org.firstinspires.ftc.teamcode.Robot.Constants;
import org.firstinspires.ftc.teamcode.Robot.Systems.*;
import org.firstinspires.ftc.teamcode.Robot.Sensing.*;


@TeleOp(name = "TeleOp - Main")
public class Tele1  extends OpMode {
    //hardware
    public DcMotorEx leftFront, leftBack, rightFront, rightBack;
    public CRServo leftwinch, rightwinch;
    public Lifts lifts;
    public Intake intake;
    public Deposit deposit;
    public VoltageSensor voltageSensor;
    public LimelightBaseSystem limelight;
    public Timer voltageSensorTimer;

    //Variables
    public boolean isIntakeReadyToTransfer = false;
    public boolean lastGamepad1LeftBumperState = false;
    private boolean areIntakeServosReady = false;
    int currentState = 1;
    public double speed = 1.0;
    private double initialVoltage;




    @Override
    public void init() {
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
        leftwinch = hardwareMap.get(CRServo.class,"left_winch");
        rightwinch = hardwareMap.get(CRServo.class,"right_winch");

        limelight = new LimelightBaseSystem(hardwareMap, "limelight");
        limelight.startLimelight();

        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        initialVoltage = voltageSensor.getVoltage();
        voltageSensorTimer = new Timer();

        lifts = new Lifts(hardwareMap,voltageSensor.getVoltage());
        intake = new Intake(hardwareMap);
        deposit = new Deposit(hardwareMap);
        CommandScheduler.getInstance().reset();

    }

    @Override
    public void init_loop() {

    }


    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        gp2_lb();
        double intakePos = intake.getExtensionPosition();
        double time = voltageSensorTimer.getTimeMillis();
        double voltage = 0;
        // need to figure out what to do with this
        if (time % 100 == 0) {
            double  newVoltage = voltageSensor.getVoltage();
            if (newVoltage >= (initialVoltage - 1.0)){
                voltage = newVoltage;
            }
        }
        //auto extends winch once end game starts
        if (120*1e3 - voltageSensorTimer.getTimeMillis() < 30*1e3){
            rightwinch.setPower(1);
            leftwinch.setPower(1);
        }

        if (gamepad1.a) {
            speed = 0.4;
            areIntakeServosReady = false;
            new SequentialCommandGroup(
                    new IntakeExtensionControlCommand(intake, Constants.intakeExtensionValues[1]),
                    new WaitCommand(200), // Wait before extension
            new InstantCommand(() -> { // Step 1: Set first batch of servos
                        intake.setServoPosition(1,0.8);
                        intake.setServoPosition(2, 0.2);
                    }),
                    new WaitCommand(200), // Wait
                    new InstantCommand(() -> { // Step 2: Set next batch of servos
                        intake.setServoPosition(3, Constants.intakeActive[2]);
                        intake.setServoPosition(4, Constants.intakeActive[3]);
                    }),
                    new WaitCommand(200), // Wait
                    new InstantCommand(() -> { // Step 3: Set final servo (e.g., claw)
                        intake.setServoPosition(6, Constants.intakeActive[4]); // Corrected index if for claw
                    })
            ).schedule();
        }

        //if (intakepos > 300 && !areIntakeServosReady) {
          //  new IntakeActiveCommand(intake).schedule();
           // areIntakeServosReady = true;
       // }


        if(gamepad1.b){
            speed = 1.0;
            new SequentialCommandGroup(new IntakeGrabCommand(intake), new WaitCommand(200), new InstantCommand(()-> intake.setServoPosition(6,0.45)),new IntakeExtensionControlCommand(intake, Constants.intakeExtensionValues[0])).schedule();
        }

        // experimental stages
        /*if(isIntakeReadyToTransfer){
            limelight.FetchResults(); // need to change this, but can only do that when transfer is finalized
            if(limelight.angleOfBlock()>3000){
                new SequentialCommandGroup(new SampleSwitchCommand(intake,deposit, lifts),new WaitCommand(150), new DepositDropPositionCommand(deposit));
            }
        }*/

        if(gamepad1.x){
            new SequentialCommandGroup(new SampleSwitchCommand(intake,deposit),new WaitCommand(150), new InstantCommand(() -> deposit.setServoPosition(4, Constants.outtakeSampleDrop[3]), deposit)).schedule();
        }

        if(gamepad1.dpad_down){
            new IntakeHorizontalSpinCommand(intake).schedule();

        }

        if(gamepad1.y){
            deposit.setServoPosition(1, Constants.outtakeSampleDrop[0]);
            deposit.setServoPosition(2, Constants.outtakeSampleDrop[1]);
            deposit.setServoPosition(3, Constants.outtakeSampleDrop[2]);

        }

        if(gamepad1.right_bumper){
            rightwinch.setPower(-1);
            leftwinch.setPower(-1);
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

    public void gp2_lb() {
        boolean currentGamepad1LeftBumperState = gamepad1.left_bumper; // Read current state once

        // Check for a rising edge: was not pressed, but now is pressed
        if (currentGamepad1LeftBumperState && !lastGamepad1LeftBumperState) {
            // The bumper was just pressed
            if (currentState == 1) {
                new PDFLCommand(lifts, 1400).schedule();
                currentState = 2; // Toggle to the next state
                telemetry.addLine("Lift moving to state 2 (e.g., High Position)");
            } else { // currentState is 2 (or any other state that should go back to 1)
                new InstantCommand(()-> deposit.setServoPosition(4,Constants.outtakeSampleReset[3]));
                new WaitCommand(200);
                new DepositResetCommand(deposit).schedule();
                new PDFLCommand(lifts, 0).schedule();
                currentState = 1; // Toggle back to the initial state
                telemetry.addLine("Lift moving to state 1 (e.g., Low Position)");
            }
        }

        // Update the last known state of the bumper for the next loop iteration
        lastGamepad1LeftBumperState = currentGamepad1LeftBumperState;
    }
}
