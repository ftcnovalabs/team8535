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
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRColor;
import org.ftcTeam.configurations.FTCTeamRobot;
import org.ftcTeam.configurations.Team7394;
import org.ftcbootstrap.ActiveOpMode;
import org.ftcbootstrap.components.operations.motors.TankDriveToEncoder;
import org.ftcbootstrap.components.operations.motors.MotorToEncoder;
import org.ftcbootstrap.components.utils.DriveDirection;
import org.ftcbootstrap.components.utils.MotorDirection;


/**
 * Note:  It is assumed that the proper registrar is used for this set of demos. To confirm please
 * search for "Enter your custom registrar"  in  {@link org.ftcTeam.FTCTeamControllerActivity}
 * <p/>
 */

@Autonomous
public class BasicAutonomousRed extends ActiveOpMode {

    private MotorToEncoder motorToEncoder;
    private FTCTeamRobot robot;
    private int turnNinetyDegrees = 2500;
    private TankDriveToEncoder tankDriveToEncoder;
    // Initial state (robot at start, not moving)
    private boolean state0 = true;
    // Knock of cap ball
    private boolean state1 = false;
    // turn right to face corner vortex
    private boolean state2 = false;
    //Drive forward to park on corner vortex
    private boolean state3 = false;
    //Shoot Balls

    /**
     * Implement this method to define the code to run when the Init button is pressed on the Driver station.
     */
    @Override
    protected void onInit() {

        robot = FTCTeamRobot.newConfig(hardwareMap, getTelemetryUtil());
        tankDriveToEncoder = new TankDriveToEncoder(this, robot.motor1, robot.motor2);
        motorToEncoder = new MotorToEncoder(this, robot.motor3);

        //  isBlue = true;
    }

    /**
     * Implement this method to define the code to run when the Start button is pressed on the Driver station.
     * This method will be called on each hardware cycle just as the loop() method is called for event based Opmodes
     */
    @Override
    protected void activeLoop() throws InterruptedException {

        if (state0) {
            boolean targetReached = false;
            //drive forward at .65 power until 10000 encoder position
            double power = .65;
            int targetEncoderValue = 10000;
            DriveDirection direction = DriveDirection.DRIVE_FORWARD;
            DcMotor.RunMode mode = DcMotor.RunMode.RUN_USING_ENCODER;


            targetReached =
                    tankDriveToEncoder.runToTarget(power, targetEncoderValue, direction, mode);

            if (targetReached) {

                //setOperationsCompleted();
                state1 = true;
                state0 = false;
            }
        } else if (state1) {
            DcMotor.RunMode mode = DcMotor.RunMode.RESET_ENCODERS;
            boolean targetReached = false;
            //drive forward at .65 power until 10000 encoder position
            double power = .65;
            int targetEncoderValue = turnNinetyDegrees+1250;
            DriveDirection direction = DriveDirection.SPIN_LEFT;
            mode = DcMotor.RunMode.RUN_USING_ENCODER;

            targetReached =
                    tankDriveToEncoder.runToTarget(power, targetEncoderValue, direction, mode);

            if (targetReached) {


                state2 = true;
                state1 = false;
            }
        } else if (state2) {
            boolean targetReached = false;
            //drive forward at .65 power until 10000 encoder position
            double power = .65;
            int targetEncoderValue = 13000;
            DriveDirection direction = DriveDirection.DRIVE_FORWARD;
            DcMotor.RunMode mode = DcMotor.RunMode.RUN_USING_ENCODER;


            targetReached =
                    tankDriveToEncoder.runToTarget(power, targetEncoderValue, direction, mode);

            if (targetReached) {

                state3 = true;;
                state2 = false;
            }
        }else if (state3) {
            boolean targetReached = false;
            //drive forward at .65 power until 10000 encoder position
            double power = .65;
            int targetEncoderValue = 500;
            MotorDirection direction = MotorDirection.MOTOR_BACKWARD;
            DcMotor.RunMode mode = DcMotor.RunMode.RUN_USING_ENCODER;


            targetReached =
                    motorToEncoder.runToTarget(power, targetEncoderValue, direction, mode);

            if (targetReached) {


                state3 = false;
            }
        }
    }
}

