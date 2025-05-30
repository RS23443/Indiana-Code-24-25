package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Testing;
import static org.firstinspires.ftc.teamcode.Robot.Constants.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.Robot.Systems.*;
@TeleOp (name = "Servo Tuner")
public class ServoTuner extends LinearOpMode {
    public Intake intake;
    public Deposit deposit;


    @Override
    public void runOpMode() throws InterruptedException {
        intake = new Intake(hardwareMap);
        deposit = new Deposit(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {

            // intake should be fully retracted and ready to grab
        if(gamepad1.a){ // should be lowest pose to grab
            intake.setServoPosition(1,1);
            intake.setServoPosition(2,0);
            intake.setServoPosition(3,0.5);
            intake.setServoPosition(4,0.5);
            intake.setServoPosition(5,0.7); // claw should close towards 0.0
        }

        if(gamepad1.b){
            intake.setServoPosition(1,0.66);
            intake.setServoPosition(2,0.34);
            intake.setServoPosition(3,0.3);
            intake.setServoPosition(4,0.7);
            intake.setServoPosition(5,0.45); // claw should close towards 0.0
        }

        if(gamepad1.x){
            intake.setServoPosition(3,0.18);
            intake.setServoPosition(4,0.18);
            //sleep(1000);
            //intake.setServoPosition(3,0);//bring up
            //intake.setServoPosition(4,1);
        }

        if(gamepad1.y){
            intake.setServoPosition(5,0.7); //open
            sleep(1000);
            intake.setServoPosition(5,0.45); // close
        }

            if (gamepad2.a) { //grab position
                deposit.setServoPosition(1, 0);
                deposit.setServoPosition(2, 1);
                deposit.setServoPosition(3, 1.0);
                deposit.setServoPosition(4, 0.8);// need to test
            }

            if (gamepad2.b) {
                deposit.setServoPosition(1, 0.3);
                deposit.setServoPosition(2, 0.7);
                deposit.setServoPosition(3, 0.35);
                deposit.setServoPosition(4, 0.63); // close
            }

            if (gamepad2.x) {
                deposit.setServoPosition(4, 0.95);
            }

        }
    }
}
