package org.firstinspires.ftc.teamcode.Robot.Sensing;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ColorSensors{
    private ColorSensor leftColorSensor;
    private ColorSensor rightColorSensor;
    private ColorSensor clawColorSensor;
    private int LCSalpha, LCSred, LCSblue, LCSgreen;
    private int RCSalpha, RCSred, RCSblue, RCSgreen;
    private int CCSalpha, CCSred, CCSblue, CCSgreen;
    public ColorSensors(final HardwareMap hardwareMap){
        leftColorSensor = hardwareMap.get(ColorSensor.class,"leftColorSensor");
        rightColorSensor = hardwareMap.get(ColorSensor.class,"rightColorSensor");
        clawColorSensor = hardwareMap.get(ColorSensor.class,"clawColorSensor");

    }

    public int RCSAlpha(){
        RCSalpha = rightColorSensor.alpha();
        return RCSalpha;
    }
    public int RCSRed(){
        RCSred = rightColorSensor.red();
        return RCSred;
    }
    public int RCSBlue(){
        RCSblue = rightColorSensor.blue();
        return RCSblue;
    }
    public int RCSGreen(){
        RCSgreen = rightColorSensor.green();
        return RCSgreen;
    }
    public int CCSAlpha(){
        CCSalpha = clawColorSensor.alpha();
        return CCSalpha;
    }
    public int CCSRed(){
        CCSred = clawColorSensor.red();
        return CCSred;
    }
    public int CCSBlue(){
        CCSblue = clawColorSensor.blue();
        return CCSblue;
    }
    public int CCSGreen(){
        CCSgreen = clawColorSensor.green();
        return CCSgreen;
    }
    public int LCSAlpha(){
        LCSalpha = leftColorSensor.alpha();
        return LCSalpha;
    }
    public int LCSRed(){
        LCSred = leftColorSensor.red();
        return LCSred;
    }
    public int LCSBlue(){
        LCSblue = leftColorSensor.blue();
        return LCSblue;
    }
    public int LCSGreen(){
        LCSgreen = leftColorSensor.green();
        return LCSgreen;
    }
}
