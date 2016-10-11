package org.ftcTeam.opmodes.registrar1;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.ftcTeam.configurations.MotorAndServoRobot;
import org.ftcTeam.configurations.Team7394;
import org.ftcbootstrap.ActiveOpMode;
import org.ftcbootstrap.components.operations.motors.MotorToEncoder;
import org.ftcbootstrap.components.operations.motors.TankDriveToEncoder;
import org.ftcbootstrap.components.utils.DriveDirection;
import org.ftcbootstrap.components.utils.MotorDirection;


/**
 * Note:  It is assumed that the proper registrar is used for this set of demos. To confirm please
 * search for "Enter your custom registrar"  in  {@link org.ftcTeam.FTCTeamControllerActivity}
 * <p/>
 */

@Autonomous
public class TestAutonomous extends ActiveOpMode {

    private Team7394 robot;
    private TankDriveToEncoder tankDriveToEncoder;


    /**
     * Implement this method to define the code to run when the Init button is pressed on the Driver station.
     */
    @Override
    protected void onInit() {

        robot = Team7394.newConfig(hardwareMap, getTelemetryUtil());
        tankDriveToEncoder = new TankDriveToEncoder(this, robot.motor1, robot.motor2);


    }

    /**
     * Implement this method to define the code to run when the Start button is pressed on the Driver station.
     * This method will be called on each hardware cycle just as the loop() method is called for event based Opmodes
     */
    @Override
    protected void activeLoop() throws InterruptedException {

        boolean targetReached = false;
        //drive forward at .65 power until 10000 encoder position
        double power = .65;
        int targetEncoderValue = 100000;
        DriveDirection direction =  DriveDirection.DRIVE_FORWARD;
        DcMotor.RunMode mode = DcMotor.RunMode.RUN_TO_POSITION;

        targetReached =
                tankDriveToEncoder.runToTarget(power, targetEncoderValue, direction, mode);

        if (targetReached) {
            setOperationsCompleted();
        }


    }

}
