package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Systems.Intake;
import org.firstinspires.ftc.teamcode.Robot.Systems.Lifts;

@TeleOp
public class EncoderReset extends LinearOpMode {
    public Lifts lifts;
    public Intake intake;
    @Override
    public void runOpMode() throws InterruptedException {
        intake = new Intake(hardwareMap);
        lifts = new Lifts(hardwareMap,12.26);
        waitForStart();
        while(opModeIsActive()){
            intake.resetExtension();
            lifts.stopandreset();
        }
    }
}
