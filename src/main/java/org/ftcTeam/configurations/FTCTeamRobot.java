package org.ftcTeam.configurations;

import android.graphics.Color;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

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
    public OpticalDistanceSensor rangeFinder;


    //motors
    public DcMotor motor1;
    public DcMotor motor2;
    public DcMotor spinner;
    public DcMotor scissorLeft;
    public DcMotor scissorRight;
    public DcMotor forkLift;


    //servos
  //  public Servo armLeft;
   // public Servo armRight;




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

            rangeFinder = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeFinder");
        //rangeFinder = (OpticalDistanceSensor) getHardwareOn("rangeFinder", hardwareMap.opticalDistanceSensor);

            motor1 = (DcMotor) getHardwareOn("motor1", hardwareMap.dcMotor);
            motor2 = (DcMotor) getHardwareOn("motor2", hardwareMap.dcMotor);
        if (motor2 != null)
            motor2.setDirection(DcMotor.Direction.REVERSE);
         spinner = (DcMotor) getHardwareOn("spinner", hardwareMap.dcMotor);
            scissorLeft = (DcMotor) getHardwareOn("scissorLeft", hardwareMap.dcMotor);
            scissorRight = (DcMotor) getHardwareOn("scissorRight", hardwareMap.dcMotor);
        if (scissorRight != null)
            scissorRight.setDirection(DcMotor.Direction.REVERSE);
        forkLift = (DcMotor) getHardwareOn("forkLift", hardwareMap.dcMotor);

//            armLeft = (Servo) getHardwareOn("armLeft", hardwareMap.servo);
  //          armRight = (Servo) getHardwareOn("armRight", hardwareMap.servo);
    //    if (armRight != null)
      //      armRight.setDirection(Servo.Direction.REVERSE);


    }




}
