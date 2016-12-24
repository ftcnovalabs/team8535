package org.ftcTeam.configurations;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.ftcbootstrap.RobotConfiguration;
import org.ftcbootstrap.components.utils.TelemetryUtil;

/**
 * FTCTeamRobot Saved Configuration
 * <p/>
 * It is assumed that there is a configuration on the phone running the Robot Controller App with the same name as this class and
 * that  configuration is the one that is marked 'activated' on the phone.
 * It is also assumed that the device names in the 'init()' method below are the same  as the devices named for the
 * saved configuration on the phone.
 */
public class FTCTeamRobot extends RobotConfiguration {

    //sensors
    //public TouchSensor touch;
    public ColorSensor color0;
    public ColorSensor color1;
    public ColorSensor color2;
    public ColorSensor colorFloor;


    //motors
    public DcMotor motor1;
    public DcMotor motor2;
    public DcMotor motor3;
  //  public Servo clamp_left;
   // public Servo clamp_right;
    public Object RunMode;

    /**
     * Factory method for this class
     *
     * @param hardwareMap
     * @param telemetryUtil
     * @return
     */
    public static FTCTeamRobot newConfig(HardwareMap hardwareMap, TelemetryUtil telemetryUtil) {

        FTCTeamRobot config = new FTCTeamRobot();
        config.init(hardwareMap, telemetryUtil);
        return config;

    }

    /**
     * Assign your class instance variables to the saved device names in the hardware map
     *
     * @param hardwareMap
     * @param telemetryUtil
     */
    @Override
    protected void init(HardwareMap hardwareMap, TelemetryUtil telemetryUtil) {

        setTelemetry(telemetryUtil);

        //touch = (TouchSensor) getHardwareOn("touch", hardwareMap.touchSensor);
        color0 = (ColorSensor) getHardwareOn("color0", hardwareMap.colorSensor);
        color1 = (ColorSensor) getHardwareOn("color1", hardwareMap.colorSensor);
        color2 = (ColorSensor) getHardwareOn("color2", hardwareMap.colorSensor);
        colorFloor = (ColorSensor) getHardwareOn("colorFloor", hardwareMap.colorSensor);
        //clamp_left = (Servo) getHardwareOn("clamp_left", hardwareMap.servo);
        //clamp_right = (Servo) getHardwareOn("clamp_right", hardwareMap.servo);
        motor1 = (DcMotor) getHardwareOn("motor1", hardwareMap.dcMotor);
        motor2 = (DcMotor) getHardwareOn("motor2", hardwareMap.dcMotor);
        motor2.setDirection(DcMotor.Direction.REVERSE);
        motor3 = (DcMotor) getHardwareOn("motor3", hardwareMap.dcMotor);


    }




}
