/*boolean targetReached = false;
        //drive forward at motorPower power until 10000 encoder position
        double power = motorPower;
        int targetEncoderValue = 2500;
        DriveDirection direction;
        if (isRed) {
        direction = DriveDirection.SPIN_RIGHT;
        } else {
        direction = DriveDirection.SPIN_LEFT;
        }
        DcMotor.RunMode mode = DcMotor.RunMode.RUN_USING_ENCODER;
*/
//motor 2 is spinning forward


package org.ftcTeam.opmodes.registrar2;
// Spin turn left on 10000 encoder values is ~360 degree turn.
import android.service.dreams.DreamService;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRColor;
import org.ftcTeam.State;
import org.ftcTeam.configurations.FTCTeamRobot;
import org.ftcTeam.configurations.Team7394;
import org.ftcbootstrap.ActiveOpMode;
import org.ftcbootstrap.components.ColorSensorComponent;
import org.ftcbootstrap.components.ODSComponent;
import org.ftcbootstrap.components.operations.motors.LineFollowToRangeFinder;
import org.ftcbootstrap.components.operations.motors.MotorToEncoder;
import org.ftcbootstrap.components.operations.motors.TankDrive;
import org.ftcbootstrap.components.operations.motors.TankDriveToColor;
import org.ftcbootstrap.components.operations.motors.TankDriveToEncoder;
import org.ftcbootstrap.components.operations.motors.TankDriveToODS;
import org.ftcbootstrap.components.utils.DriveDirection;

import java.util.ArrayList;


/**
 * Note:  It is assumed that the proper registrar is used for this set of demos. To confirm please
 * search for "Enter your custom registrar"  in  {@link org.ftcTeam.FTCTeamControllerActivity}
 * <p/>
 */

@Autonomous
public class AutonomousLineFollow extends ActiveOpMode {
    private double motorPower = .5;
    private FTCTeamRobot robot;
    private ColorSensorComponent colorSensorComponentFloor;

    private ColorSensorComponent colorSensorComponent0;
    private ColorSensorComponent colorSensorComponent1;

    boolean sleepBetweenStates = true;
    private LineFollowToRangeFinder lineFollowToRangeFinder;
    private TankDriveToODS tankDriveToODS;
    private TankDrive tankDrive;

    private int previousR;
    private int previousG;
    private int previousB;

    private DriveDirection previousPivot;


    /**
     * Implement this method to define the code to run when the Init button is pressed on the Driver station.
     */
    @Override
    protected void onInit() {
        robot = FTCTeamRobot.newConfig(hardwareMap, getTelemetryUtil());

        tankDriveToODS = new TankDriveToODS(this, robot.rangeFinder, robot.motor1, robot.motor2);

        //tankDrive = new TankDrive(this, robot.motor2, robot.motor1);
        tankDrive = new TankDrive(this, robot.motor1, robot.motor2);

        colorSensorComponent0 = new ColorSensorComponent(this, robot.color0, ColorSensorComponent.ColorSensorDevice.MODERN_ROBOTICS_I2C);
        colorSensorComponent1 = new ColorSensorComponent(this, robot.color1, ColorSensorComponent.ColorSensorDevice.MODERN_ROBOTICS_I2C, I2cAddr.create8bit(0x42));
        colorSensorComponentFloor = new ColorSensorComponent(this, robot.colorFloor, ColorSensorComponent.ColorSensorDevice.MODERN_ROBOTICS_I2C, I2cAddr.create8bit(0x50));
        lineFollowToRangeFinder = new LineFollowToRangeFinder(this, robot.rangeFinder, DriveDirection.DRIVE_BACKWARD, colorSensorComponentFloor, true, new TankDrive(this, robot.motor2, robot.motor1), new ODSComponent(this, robot.rangeFinder));

        colorSensorComponent0.enableLed(false);
        colorSensorComponent1.enableLed(false);

        previousR = colorSensorComponentFloor.getR();
        previousG = colorSensorComponentFloor.getG();
        previousB = colorSensorComponentFloor.getB();
        previousPivot = DriveDirection.FAST_PIVOT_BACKWARD_RIGHT;
    }


    public  boolean lineFollowToTarget(LineFollowToRangeFinder _lftrf, double power, double targetProximity, DriveDirection directionToLine)
            throws InterruptedException {



        telemetry.addData("ODS: ", lineFollowToRangeFinder.getOdsReading());
        telemetry.addData("Floor B: ", colorSensorComponentFloor.getB());
        telemetry.addData("Floor R: ", colorSensorComponentFloor.getR());
        telemetry.addData("Floor G: ", colorSensorComponentFloor.getG());

        //this.targetValue = targetProximity;
        telemetry.addLine("tankDrive.isDriving() == "+tankDrive.isDriving());
        if (tankDrive.isDriving()) {

            boolean distanceReached =  _lftrf.targetReached();
            telemetry.addData("_lfrtf.targetReached() = ", _lftrf.targetReached());

            if (distanceReached) {
                telemetry.addLine("_distanceReached == true");
                stop();
                return distanceReached;
            }

            if (_lftrf.lineTargetReached()) {
                previousPivot = tankDrive.getCurrentDirection();
                tankDrive.drive(power, DriveDirection.DRIVE_BACKWARD);
            } else if ( colorSensorComponentFloor.getR() < previousR ||
                        colorSensorComponentFloor.getG() < previousG ||
                        colorSensorComponentFloor.getB() < previousB) {
                if ( previousPivot == DriveDirection.FAST_PIVOT_BACKWARD_LEFT) {
                    tankDrive.drive( power, DriveDirection.FAST_PIVOT_BACKWARD_RIGHT);
                }
                else {
                    tankDrive.drive( power, DriveDirection.FAST_PIVOT_BACKWARD_LEFT);
                }
            }



//            boolean reached =  _lftrf.lineTargetReached();
//            telemetry.addLine("lineTargetReached == "+reached);
//
//            if ( (reached && _lftrf.headingToLine) || (!reached && !_lftrf.headingToLine) ) {
//
//                if ( tankDrive.getCurrentDirection() == DriveDirection.FAST_PIVOT_BACKWARD_LEFT) {
//                    tankDrive.drive( power, DriveDirection.FAST_PIVOT_BACKWARD_RIGHT);
//                }
//                else {
//                    tankDrive.drive( power, DriveDirection.FAST_PIVOT_BACKWARD_LEFT);
//                }
//

                //check if rolled past line
//                if ( ! _lftrf.headingToLine ) {
//                    //Handle potent of robot rolling past line.   Allow time to get
//                    //back to or to the correct side of the line.
//                    Thread.sleep(200);
//                }
//                _lftrf.headingToLine = !_lftrf.headingToLine;
//           }
            telemetry.addData("headingToLine == ", _lftrf.headingToLine);
            telemetry.addData("motor1.pos: ", robot.motor1.getCurrentPosition());
            telemetry.addData("motor2.pos: ", robot.motor2.getCurrentPosition());
            telemetry.addData("drive direction:", tankDrive.getCurrentDirection());
            telemetry.update();
            return false;
        }

        _lftrf.headingToLine = true;
        _lftrf.setTarget(targetProximity);
        //_lftrf.odsComponent.setTarget( targetProximity);
        tankDrive.drive( power, directionToLine);
        return false;

    }

    /**
     * Implement this method to define the code to run when the Start button is pressed on the Driver station.
     * This method will be called on each hardware cycle just as the loop() method is called for event based Opmodes
     */
    @Override
    protected void activeLoop() throws InterruptedException {
        double power = 0.2;
        //boolean targetReached = tankDriveToODS.lineFollowT0RangeFinder(power,0.55,10.0,DriveDirection.PIVOT_FORWARD_RIGHT);

        // ** possible wheel wiggle issue: int targetEncoderValue = 2000;
        DriveDirection direction = DriveDirection.PIVOT_BACKWARD_RIGHT;
        //lineFollowToRangeFinder.setTarget(0.55);


        double distanceToTarget = lineFollowToRangeFinder.targetDistance();
        boolean targetReached = distanceToTarget >= 0.55;

        //lineFollowToRangeFinder.lineFollowToTarget(power, 0.55, DriveDirection.PIVOT_BACKWARD_RIGHT);

        //boolean targetReached =                lineFollowToRangeFinder.lineFollowToTarget(power, 0.55, direction);

        if (targetReached) {
            //state3 = true;
            //state2 = false;
            telemetry.addData("ODS: ", lineFollowToRangeFinder.getOdsReading());
            telemetry.addLine("state2");
            telemetry.addData("Floor B: ", colorSensorComponentFloor.getB());
            telemetry.addData("Floor R: ", colorSensorComponentFloor.getR());
            telemetry.addData("Floor G: ", colorSensorComponentFloor.getG());
            telemetry.addLine("targetReached");
            telemetry.update();
            //setOperationsCompleted();
            this.tankDrive.stop();
        } else {
            lineFollowToTarget(lineFollowToRangeFinder, power, 0.55, direction);
        }


    }
}