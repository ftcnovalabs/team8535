/*boolean targetReached = false;
        //drive forward at .65 power until 10000 encoder position
        double power = .65;
        int targetEncoderValue = 2500;
        DriveDirection direction;
        if (isBlue) {
        direction = DriveDirection.SPIN_RIGHT;
        } else {
        direction = DriveDirection.SPIN_LEFT;
        }
        DcMotor.RunMode mode = DcMotor.RunMode.RUN_USING_ENCODER;
*/



package org.ftcTeam.opmodes.registrar2;
// Spin turn left on 10000 encoder values is ~360 degree turn.
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.I2cAddr;

import org.ftcTeam.configurations.FTCTeamRobot;
import org.ftcbootstrap.ActiveOpMode;
import org.ftcbootstrap.components.ColorSensorComponent;


/**
 * Note:  It is assumed that the proper registrar is used for this set of demos. To confirm please
 * search for "Enter your custom registrar"  in  {@link org.ftcTeam.FTCTeamControllerActivity}
 * <p/>
 */

@Autonomous
public class TestSingleColorSensor extends ActiveOpMode {

    private FTCTeamRobot robot;

    private ColorSensorComponent colorSensorComponent0;
    private ColorSensorComponent colorSensorComponent1;
    private ColorSensorComponent colorSensorComponent2;

    /**
     * Implement this method to define the code to run when the Init button is pressed on the Driver station.
     */
    @Override
    protected void onInit() {

        robot = FTCTeamRobot.newConfig(hardwareMap, getTelemetryUtil());
        colorSensorComponent0 = new ColorSensorComponent(this, robot.color0, ColorSensorComponent.ColorSensorDevice.MODERN_ROBOTICS_I2C);
        //disable LED to allow sensor to measure light project from the beacon
        //colorSensorComponent1 = new ColorSensorComponent(this, robot.color1, ColorSensorComponent.ColorSensorDevice.MODERN_ROBOTICS_I2C, I2cAddr.create8bit(0x42));
        //colorSensorComponent2 = new ColorSensorComponent(this, robot.color2, ColorSensorComponent.ColorSensorDevice.MODERN_ROBOTICS_I2C, I2cAddr.create8bit(0x48));

        colorSensorComponent0.enableLed(false);
        //colorSensorComponent1.enableLed(false);
        //colorSensorComponent2.enableLed(false);


    }

    /**
     * Implement this method to define the code to run when the Start button is pressed on the Driver station.
     * This method will be called on each hardware cycle just as the loop() method is called for event based Opmodes
     */
    @Override
    protected void activeLoop() throws InterruptedException {

        telemetry.addData("0state3 B: ", colorSensorComponent0.getB());
        telemetry.addData("0state3 R: ", colorSensorComponent0.getR());
        telemetry.addData("0state3 G: ", colorSensorComponent0.getG());

        //telemetry.addData("1state3 B: ", colorSensorComponent1.getB());
        //telemetry.addData("1state3 R: ", colorSensorComponent1.getR());
        //telemetry.addData("1state3 G: ", colorSensorComponent1.getG());

        //telemetry.addData("2state3 B: ", colorSensorComponent2.getB());
        //telemetry.addData("2state3 R: ", colorSensorComponent2.getR());
        //telemetry.addData("2state3 G: ", colorSensorComponent2.getG());



        telemetry.update();


    }
}