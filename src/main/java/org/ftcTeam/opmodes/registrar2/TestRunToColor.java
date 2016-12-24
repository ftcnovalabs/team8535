package org.ftcTeam.opmodes.registrar2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.ftcTeam.configurations.FTCTeamRobot;
import org.ftcbootstrap.ActiveOpMode;
import org.ftcbootstrap.components.ColorSensorComponent;
import org.ftcbootstrap.components.operations.motors.TankDriveToColor;
import org.ftcbootstrap.components.operations.motors.TankDriveToEncoder;
import org.ftcbootstrap.components.utils.DriveDirection;
import com.qualcomm.robotcore.hardware.I2cAddr;

/**
 * Note:  It is assumed that the proper registrar is used for this set of demos. To confirm please
 * search for "Enter your custom registrar"  in  {@link org.ftcTeam.FTCTeamControllerActivity}
 * <p/>
 */

@Autonomous
public class TestRunToColor extends ActiveOpMode {

    private FTCTeamRobot robot;
    private TankDriveToEncoder tankDriveToEncoder;
    private TankDriveToColor tankDriveToColor;
    private ColorSensorComponent colorSensorComponentFloor;


    /**
     * Implement this method to define the code to run when the Init button is pressed on the Driver station.
     */
    @Override
    protected void onInit() {

        robot = FTCTeamRobot.newConfig(hardwareMap, getTelemetryUtil());
        colorSensorComponentFloor = new ColorSensorComponent(this, robot.colorFloor, ColorSensorComponent.ColorSensorDevice.MODERN_ROBOTICS_I2C, I2cAddr.create8bit(0x50));
        //tankDriveToEncoder = new TankDriveToEncoder(this, robot.motor1, robot.motor2);
        tankDriveToColor = new TankDriveToColor(this, colorSensorComponentFloor,robot.motor1,robot.motor2);

    }

    /**
     * Implement this method to define the code to run when the Start button is pressed on the Driver station.
     * This method will be called on each hardware cycle just as the loop() method is called for event based Opmodes
     */
    @Override
    protected void activeLoop() throws InterruptedException {

        boolean targetReached = false;
        //drive forward until color reached
        double power = 1.0;
        int targetR = 50;
        int targetB = 50;
        int targetG = 50;
        DriveDirection direction =  DriveDirection.DRIVE_BACKWARD;
        DcMotor.RunMode mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;

        targetReached = tankDriveToColor.runToTarget(power, targetR, targetB, targetG, direction);

        telemetry.addData("Floor R: ", colorSensorComponentFloor.getR());
        telemetry.addData("Floor G: ", colorSensorComponentFloor.getG());
        telemetry.addData("Floor B: ", colorSensorComponentFloor.getB());

        telemetry.update();

        if (targetReached) {
            setOperationsCompleted();
        }


    }


}
