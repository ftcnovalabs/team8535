package org.ftcTeam.opmodes.registrar2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.ftcTeam.configurations.FTCTeamRobot;
import org.ftcbootstrap.ActiveOpMode;
import org.ftcbootstrap.components.ColorSensorComponent;
import org.ftcbootstrap.components.ServoComponent;
import org.ftcbootstrap.components.operations.motors.TankDriveToEncoder;
import org.ftcbootstrap.components.operations.motors.TankDriveToTime;
import org.ftcbootstrap.components.utils.DriveDirection;

/**
 * Note:  It is assumed that the proper registrar is used for this set of demos. To confirm please
 * search for "Enter your custom registrar"  in  {@link org.ftcTeam.FTCTeamControllerActivity}
 * <p/>
 */

@Autonomous
public abstract class AutonomousBeacon extends ActiveOpMode {

    private FTCTeamRobot robot;

    private TankDriveToEncoder tankDriveToEncoder;
    private TankDriveToTime tankDriveToTime;
    private ServoComponent leftServoComponent;
    private ServoComponent rightServoComponent;

    private ColorSensorComponent colorSensorComponent;
    private ColorSensorComponent colorSensorComponent1;
    private int step;

    protected abstract boolean isRedAlliance();

    /**
     * Implement this method to define the code to run when the Init button is pressed on the Driver station.
     */
    @Override
    protected void onInit() {

        robot = FTCTeamRobot.newConfig(hardwareMap, getTelemetryUtil());

        tankDriveToEncoder = new TankDriveToEncoder(this, robot.motor1, robot.motor2);
        tankDriveToTime = new TankDriveToTime(this, robot.motor1, robot.motor2);

       // colorSensorComponent = new ColorSensorComponent(this, robot.color01, ColorSensorComponent.ColorSensorDevice.MODERN_ROBOTICS_I2C);
        //disable LED to allow sensor to measure light project from the beacon
        colorSensorComponent.enableLed(false);

        step = 1;
    }


    /**
     * Implement this method to define the code to run when the Start button is pressed on the Driver station.
     * This method will be called on each hardware cycle just as the loop() method is called for event based Opmodes
     *
     * @throws InterruptedException
     */
    @Override
    protected void activeLoop() throws InterruptedException {

        boolean targetReached = false;

        switch (step) {
            case 1:
                //Drive forward
                targetReached = tankDriveToEncoder.runToTarget(.65, 5500,
                        DriveDirection.DRIVE_BACKWARD, DcMotor.RunMode.RUN_TO_POSITION);
                if (targetReached) {
                    step++;
                }
                break;

            case 2:
                //turn  45 degrees
                DriveDirection direction = isRedAlliance() ? DriveDirection.PIVOT_FORWARD_RIGHT : DriveDirection.PIVOT_FORWARD_RIGHT;
                targetReached = tankDriveToEncoder.runToTarget(0.4, 2525,
                        direction, DcMotor.RunMode.RUN_TO_POSITION);
                if (targetReached) {
                    step++;
                }
                break;

            case 3:
                //full power forward
                targetReached = tankDriveToEncoder.runToTarget(.65, 9000,
                        DriveDirection.DRIVE_BACKWARD, DcMotor.RunMode.RUN_TO_POSITION);
                if (targetReached) {
                    step++;
                }
                break;

            case 4:
                //2nd turn right another  45 degrees
                direction = isRedAlliance() ? DriveDirection.PIVOT_FORWARD_LEFT : DriveDirection.PIVOT_FORWARD_RIGHT;
                targetReached = tankDriveToEncoder.runToTarget(0.4, 2525,
                        direction, DcMotor.RunMode.RUN_TO_POSITION);
                if (targetReached) {
                    step++;
                }
                break;


            case 5:
                //drive close to beacon
                targetReached = tankDriveToEncoder.runToTarget(.65, 5500,
                        DriveDirection.DRIVE_BACKWARD, DcMotor.RunMode.RUN_TO_POSITION);
                if (targetReached) {
                    step = 7;
                }
                break;

            case 6:
                //drive into beacon with TIME
                targetReached = tankDriveToTime.runToTarget(0.5, 0, DriveDirection.DRIVE_BACKWARD);
                if (targetReached) {
                    step++;
                }
                break;

            case 7:
                //drive backward a bit
                targetReached = tankDriveToEncoder.runToTarget(.65, 400,
                        DriveDirection.DRIVE_FORWARD, DcMotor.RunMode.RUN_TO_POSITION);
                if (targetReached) {
                    step++;
                }
                break;


            case 8:
                //Detect beacon

                boolean beaconColorIsBlue = colorSensorComponent.isBlue(10, 10, 10)|| colorSensorComponent1.isBlue(10, 10, 10);
                    telemetry.addData("8: Red  ", colorSensorComponent.getR());
                telemetry.addData("8: Blue ", colorSensorComponent.getB());

                telemetry.addData("8: IsRed  ", ! beaconColorIsBlue);
                telemetry.addData("8: IsBlue ", beaconColorIsBlue);

                //send any telemetry that may have been added in the above operations
                telemetry.update();

                //if color is the same as my alliance then set step to 11 to end the opmode
                //else got to the next step to wait 5 seconds, back up and try again
                if ((isRedAlliance()) && ! beaconColorIsBlue) {
                    telemetry.addData("8: Color matches Alliance (Red) ", isRedAlliance());
                    //send any telemetry that may have been added in the above operations
                    telemetry.update();
                    targetReached = tankDriveToTime.runToTarget(0.5, 1, DriveDirection.PIVOT_BACKWARD_RIGHT);
                    if (targetReached) {
                        step++;
                    }
                    break;

                } else if (isRedAlliance() == false && beaconColorIsBlue) {
                    telemetry.addData("8: Color matches Alliance (Blue)", !isRedAlliance());
                    //send any telemetry that may have been added in the above operations
                    telemetry.update();
                    targetReached = tankDriveToTime.runToTarget(0.5, 1, DriveDirection.PIVOT_BACKWARD_RIGHT);
                    if (targetReached) {
                            step++;
                        }
                        break;
                    } else {
                    targetReached = tankDriveToTime.runToTarget(0.5, 1, DriveDirection.PIVOT_BACKWARD_LEFT);
                    if (targetReached) {
                        step++;
                    }
                }
                            break;

                    case 9:
                        //back up from beacon
                        targetReached = tankDriveToEncoder.runToTarget(.65, 1000,
                                DriveDirection.DRIVE_FORWARD, DcMotor.RunMode.RUN_TO_POSITION);
                        if (targetReached) {
                            step++;
                        }
                        break;
                            //break;

                        /*case 9:
                            // we are not using sleep() but rather checking an elapsed time
                            // therefore the the above kill switch will work whenever you press it/
                            //See the next example that incorportates the timer into the motor operation
                            if (getTimer().targetReached(5)) {
                                step++;
                            }

                        case 10:
                            //drive into beacon with TIME
                            targetReached = tankDriveToTime.runToTarget(0.5, 2, DriveDirection.DRIVE_BACKWARD);
                            if (targetReached) {
                                step++;
                            }
                            break;

*/



/*
            case 12:
                //turn 90 decrees
                targetReached = tankDriveToEncoder.runToTarget(.65, 1000,
                        DriveDirection.DRIVE_FORWARD,DcMotor.RunMode.RUN_TO_POSITION);
                if (targetReached) {
                    step++;
                }
                break;



            case 13:
                //drive foreword to 2nt beacon
                targetReached = tankDriveToEncoder.runToTarget(.65, 1000,
                        DriveDirection.DRIVE_FORWARD,DcMotor.RunMode.RUN_TO_POSITION);
                if (targetReached) {
                    step++;
                }
                break;
 */

            default:
                setOperationsCompleted();
                break;


                        }
                    }
                }





