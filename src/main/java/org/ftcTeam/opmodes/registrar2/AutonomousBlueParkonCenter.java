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
import org.ftcbootstrap.components.utils.DriveDirection;


/**
 * Note:  It is assumed that the proper registrar is used for this set of demos. To confirm please
 * search for "Enter your custom registrar"  in  {@link org.ftcTeam.FTCTeamControllerActivity}
 * <p/>
 */

@Autonomous
public class AutonomousBlueParkonCenter extends ActiveOpMode {

    private FTCTeamRobot robot;

    private boolean isBlue;
    private boolean firstBeaconIsTeamColor;
    // define states of robot
    private TankDriveToEncoder tankDriveToEncoder;
    // Initial state (robot at start, not moving)
    private boolean state0 = true;
    // Aligned with First Beacon
    private boolean state1 = false;
    // Turned left to face first beacon
    private boolean state2 = false;
    //press first beacon button(s)
    private boolean state3 = false;
    // Check Color Sensor then store color value
    private boolean state4 = false;
    //knock of Cap ball
    private boolean state5 = false;
    //face second beacon
    private boolean state6 = false;
    //approach second beacon
    private boolean state7 = false;
    //face second beacon
    private boolean state8 = false;
    //press second beacon button(s)
    private boolean state9Correct = false;
    // Check Color Sensor then store color value
    private boolean state9InCorrect = false;
    //Drive backward wait 5 seconds then drive forward to switch beacon color
    private boolean state10 = false;
    //clear robot to turn
    private boolean state11 = false;
    //prepare to travel to first beacon
    private boolean state12InCorrect = false;
    //align with first beacon and press first beacon button(s)
    private boolean state12Correct = false;
    // Clear robot to turn
    private boolean state13 = false;
    // face corner vortex
    private boolean state14 = false;
    //park on corner vortex

    /**
     * Implement this method to define the code to run when the Init button is pressed on the Driver station.
     */
    @Override
    protected void onInit() {

        robot = FTCTeamRobot.newConfig(hardwareMap, getTelemetryUtil());
        tankDriveToEncoder = new TankDriveToEncoder(this, robot.motor1, robot.motor2);
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
            DriveDirection direction = DriveDirection.DRIVE_BACKWARD;
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
            int targetEncoderValue = 2500;
            DriveDirection direction = DriveDirection.SPIN_LEFT;
            mode  = DcMotor.RunMode.RUN_USING_ENCODER;

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
            int targetEncoderValue = 10000;
            DriveDirection direction = DriveDirection.DRIVE_BACKWARD;
            DcMotor.RunMode mode = DcMotor.RunMode.RUN_USING_ENCODER;


            targetReached =
                    tankDriveToEncoder.runToTarget(power, targetEncoderValue, direction, mode);

            if (targetReached) {


                state3 = true;
                state2 = false;
            }
        } else if (state3) {
            /** TODO: Learn how to read the color sensor **/
            boolean firstBeaconIsTeamColor = true;
            // getTelemetryUtil(robot.color);

            if (firstBeaconIsTeamColor) {


                state4 = true;
                state3 = false;
            }
        } else if (state4) {
            boolean targetReached = false;
            //drive forward at .65 power until 10000 encoder position
            double power = .65;
            int targetEncoderValue = 10000;
            DriveDirection direction = DriveDirection.DRIVE_FORWARD;
            DcMotor.RunMode mode = DcMotor.RunMode.RUN_USING_ENCODER;


            targetReached =
                    tankDriveToEncoder.runToTarget(power, targetEncoderValue, direction, mode);

            if (targetReached) {

                state5 = true;
                state4 = false;
            }
        }



        else if (state5) {
            boolean targetReached = false;
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


            targetReached =
                    tankDriveToEncoder.runToTarget(power, targetEncoderValue, direction, mode);

            if (targetReached) {

                state6 = true;
                state5 = false;
            }
        } else if (state6) {
            boolean targetReached = false;
            //drive forward at .65 power until 10000 encoder position
            double power = .65;
            int targetEncoderValue = 10000;
            DriveDirection direction = DriveDirection.DRIVE_BACKWARD;
            DcMotor.RunMode mode = DcMotor.RunMode.RUN_USING_ENCODER;


            targetReached =
                    tankDriveToEncoder.runToTarget(power, targetEncoderValue, direction, mode);

            if (targetReached) {

                state7 = true;
                state6 = false;
            }
        } else if (state7) {
            boolean targetReached = false;
            //drive forward at .65 power until 10000 encoder position
            double power = .65;
            int targetEncoderValue = 2500;
            DriveDirection direction;
            if (isBlue) {
                direction = DriveDirection.SPIN_LEFT;
            } else {
                direction = DriveDirection.SPIN_RIGHT;
            }            DcMotor.RunMode mode = DcMotor.RunMode.RUN_USING_ENCODER;


            targetReached =
                    tankDriveToEncoder.runToTarget(power, targetEncoderValue, direction, mode);

            if (targetReached) {

                state8 = true;
                state7 = false;
            }
        } else if (state8) {
            boolean targetReached = false;
            //drive forward at .65 power until 10000 encoder position
            double power = .65;
            int targetEncoderValue = 2500;
            DriveDirection direction = DriveDirection.DRIVE_BACKWARD;
            DcMotor.RunMode mode = DcMotor.RunMode.RUN_USING_ENCODER;


            targetReached =
                    tankDriveToEncoder.runToTarget(power, targetEncoderValue, direction, mode);

            if (targetReached) {

                state8 = false;
                // TODO how to learn to read the color sensor
                boolean secondBeaconIsTeamColor = false;
                if (secondBeaconIsTeamColor) {
                    state9Correct = true;
                } else {
                    state9InCorrect = true;
                }
            }

        } else if (state9InCorrect) {
            boolean targetReached = false;
            //drive forward at .65 power until 10000 encoder position
            double power = .65;
            int targetEncoderValue = 2500;
            DriveDirection direction = DriveDirection.DRIVE_FORWARD;
            DcMotor.RunMode mode = DcMotor.RunMode.RUN_USING_ENCODER;


            targetReached =
                    tankDriveToEncoder.runToTarget(power, targetEncoderValue, direction, mode);

            if (targetReached) {
                sleep(5000);
                boolean targetReached2 = false;
                //drive forward at .65 power until 10000 encoder position
                double power2 = .65;
                int targetEncoderValue2 = 2500;
                DriveDirection direction2 = DriveDirection.DRIVE_BACKWARD;
                DcMotor.RunMode mode2 = DcMotor.RunMode.RUN_USING_ENCODER;


                targetReached2 =
                        tankDriveToEncoder.runToTarget(power2, targetEncoderValue2, direction2, mode2);

                if (targetReached2) {

                    state9Correct = true;
                    state9InCorrect = false;
                }
            }


        } else if (state9Correct) {
            boolean targetReached = false;
            //drive forward at .65 power until 10000 encoder position
            double power = .65;
            int targetEncoderValue = 10000;
            DriveDirection direction = DriveDirection.DRIVE_FORWARD;
            DcMotor.RunMode mode = DcMotor.RunMode.RUN_USING_ENCODER;


            targetReached =
                    tankDriveToEncoder.runToTarget(power, targetEncoderValue, direction, mode);

            if (targetReached) {

                state10 = true;
                state9Correct = false;
            }
        } else if (state10) {
            boolean targetReached = false;
            //drive forward at .65 power until 10000 encoder position
            double power = .65;
            int targetEncoderValue = 2500;
            DriveDirection direction;
            if (isBlue) {
                direction = DriveDirection.SPIN_LEFT;
            }
            else {
                direction = DriveDirection.SPIN_RIGHT;
            }            DcMotor.RunMode mode = DcMotor.RunMode.RUN_USING_ENCODER;


            targetReached =
                    tankDriveToEncoder.runToTarget(power, targetEncoderValue, direction, mode);

            if (targetReached) {

                state11 = true;
                state10 = false;
            }
        } else if (state11) {
            boolean targetReached = false;
            //drive forward at .65 power until 10000 encoder position
            double power = .65;
            int targetEncoderValue = 10000;
            DriveDirection direction = DriveDirection.DRIVE_BACKWARD;
            DcMotor.RunMode mode = DcMotor.RunMode.RUN_USING_ENCODER;


            targetReached =
                    tankDriveToEncoder.runToTarget(power, targetEncoderValue, direction, mode);

            if (targetReached) {

                state11 = false;
                // TODO how to learn to read the color sensor
                boolean secondBeaconIsTeamColor = false;
                if (firstBeaconIsTeamColor) {
                    state12Correct = true;
                } else {
                    state12InCorrect = true;
                }
            }
        } else if (state12InCorrect) {
            boolean targetReached = false;
            //drive forward at .65 power until 10000 encoder position
            double power = .65;
            int targetEncoderValue = 2500;
            DriveDirection direction;
            if (isBlue) {
                direction = DriveDirection.SPIN_RIGHT;
            }
            else {
                direction = DriveDirection.SPIN_LEFT;
            }
            DcMotor.RunMode mode = DcMotor.RunMode.RUN_USING_ENCODER;


            targetReached =
                    tankDriveToEncoder.runToTarget(power, targetEncoderValue, direction, mode);

            if (targetReached) {

                boolean targetReached2 = false;
                //drive forward at .65 power until 10000 encoder position
                double power2 = .65;
                int targetEncoderValue2 = 2500;
                DriveDirection direction2 = DriveDirection.DRIVE_BACKWARD;
                DcMotor.RunMode mode2 = DcMotor.RunMode.RUN_USING_ENCODER;


                targetReached2 =
                        tankDriveToEncoder.runToTarget(power2, targetEncoderValue2, direction2, mode2);
                if (targetReached2) {

                    state13 = true;
                    state12InCorrect = false;
                }
            }
        }
        else if (state13) {
            boolean targetReached = false;
            //drive forward at .65 power until 10000 encoder position
            double power = .65;
            int targetEncoderValue = 10000;
            DriveDirection direction = DriveDirection.DRIVE_FORWARD;
            DcMotor.RunMode mode = DcMotor.RunMode.RUN_USING_ENCODER;


            targetReached =
                    tankDriveToEncoder.runToTarget(power, targetEncoderValue, direction, mode);

            if (targetReached) {

                state13 = false;
            }
        }

            }
        }