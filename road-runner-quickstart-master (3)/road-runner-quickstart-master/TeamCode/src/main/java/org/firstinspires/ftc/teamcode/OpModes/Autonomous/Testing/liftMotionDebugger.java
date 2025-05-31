package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Robot.Sensing.Timer;
import org.firstinspires.ftc.teamcode.Robot.Sensing.PDFL;
import org.firstinspires.ftc.teamcode.Robot.Systems.Deposit;

@Config
@TeleOp
public class liftMotionDebugger extends OpMode {

    private DcMotorEx middleMotor;
    private DcMotorEx bottomMotor;
    private DcMotorEx topMotor;


    @Override
    public void init() {
        // Initialize the PDFL controller


        // Configure motors
        middleMotor = hardwareMap.get(DcMotorEx.class, "middleslide");
        bottomMotor = hardwareMap.get(DcMotorEx.class, "bottomslide");
        topMotor = hardwareMap.get(DcMotorEx.class,"topslide");
        topMotor.setDirection(DcMotorEx.Direction.REVERSE);
        bottomMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        //update voltage reading at the beginning of every loop

        if(gamepad1.a){
            middleMotor.setPower(0.2);
        }

        if(gamepad1.b){
            bottomMotor.setPower(0.2);
        }

        if(gamepad1.x){
            topMotor.setPower(0.2);
        }

        if(gamepad1.y){
            middleMotor.setPower(0);
            topMotor.setPower(0);
            bottomMotor.setPower(0);
        }

        }

        // Display the voltage on the driver station
        // Apply calculated power to motors
}
