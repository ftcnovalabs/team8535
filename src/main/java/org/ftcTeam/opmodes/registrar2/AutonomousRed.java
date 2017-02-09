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



package org.ftcTeam.opmodes.registrar2;
// Spin turn left on 10000 encoder values is ~360 degree turn.
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
import org.ftcbootstrap.components.utils.DriveDirection;

import java.util.ArrayList;


/**
 * Note:  It is assumed that the proper registrar is used for this set of demos. To confirm please
 * search for "Enter your custom registrar"  in  {@link org.ftcTeam.FTCTeamControllerActivity}
 * <p/>
 */

@Autonomous
public class AutonomousRed extends ActiveOpMode {
    private double motorPower = 1;
    private FTCTeamRobot robot;
    private TankDriveToColor tankDriveToColor;
    private ColorSensorComponent colorSensorComponentFloor;

    private ColorSensorComponent colorSensorComponentLeft;
    private ColorSensorComponent colorSensorComponentRight;
    private ColorSensorComponent colorSensorComponent2;

    boolean isBlue = false;

    private boolean firstBeaconIsTeamColor;
    // define states of robot
    private LineFollowToRangeFinder lineFollowToRangeFinder;
    // Initial state (robot at start, not moving)
    private TankDriveToEncoder tankDriveToEncoder;

    private ArrayList<State> stateMap;

    private boolean state0 = true;
    // partial turn
    private boolean state0_5 = false;
    // Aligned with First Beacon
    private boolean state0_75 = false;

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
    private boolean state6_5 = false;

    private boolean state7 = false;
    //face second beacon
    private boolean state8 = false;
    //press second beacon button(s)
    private boolean state9Correct = false;
    // Check Color Sensor then store color value
    private boolean state9Correct_5 = false;


    private boolean state9InCorrect = false;
    //Drive backward wait 5 seconds then drive forward to switch beacon color
    private boolean state9InCorrect_5 = false;

    private boolean state10 = false;
    //clear robot to turn
    private boolean state10_5 = false;


    private boolean state11 = false;
    //prepare to travel to first beacon
    private boolean state12InCorrect = false;
    //align with first beacon and press first beacon button(s)
    private boolean state12InCorrect_5 = false;

    private boolean state12InCorrect_75 = false;

    private boolean state12InCorrect_9 = false;

    private boolean state12Correct = false;


    protected void initializeStatemap() {

        stateMap = new ArrayList<State>();
        stateMap.add(new State("INITIAL", new ArrayList<State>()));
    }

    /**
     * Implement this method to define the code to run when the Init button is pressed on the Driver station.
     */
    @Override
    protected void onInit() {
        robot = FTCTeamRobot.newConfig(hardwareMap, getTelemetryUtil());
        colorSensorComponentFloor = new ColorSensorComponent(this, robot.colorFloor, ColorSensorComponent.ColorSensorDevice.MODERN_ROBOTICS_I2C, I2cAddr.create8bit(0x50));

        tankDriveToColor = new TankDriveToColor(this, colorSensorComponentFloor, robot.driveLeft, robot.driveRight);
        lineFollowToRangeFinder = new LineFollowToRangeFinder(this, robot.rangeFinder, DriveDirection.DRIVE_BACKWARD, colorSensorComponentFloor, true, new TankDrive(this, robot.driveLeft, robot.driveRight), new ODSComponent(this, robot.rangeFinder));
        tankDriveToEncoder = new TankDriveToEncoder(this, robot.driveLeft, robot.driveRight);
        colorSensorComponentLeft = new ColorSensorComponent(this, robot.colorLeft, ColorSensorComponent.ColorSensorDevice.MODERN_ROBOTICS_I2C);
        //disable LED to allow sensor to measure light project from the beacon
        colorSensorComponentRight = new ColorSensorComponent(this, robot.colorRight, ColorSensorComponent.ColorSensorDevice.MODERN_ROBOTICS_I2C, I2cAddr.create8bit(0x42));
        colorSensorComponent2 = new ColorSensorComponent(this, robot.color2, ColorSensorComponent.ColorSensorDevice.MODERN_ROBOTICS_I2C, I2cAddr.create8bit(0x48));


        colorSensorComponentLeft.enableLed(false);
        colorSensorComponentRight.enableLed(false);
        colorSensorComponent2.enableLed(false);


    }

    /**
     * Implement this method to define the code to run when the Start button is pressed on the Driver station.
     * This method will be called on each hardware cycle just as the loop() method is called for event based Opmodes
     */
    @Override
    protected void activeLoop() throws InterruptedException {

        if (state0) {
            boolean targetReached = false;
            //drive forward at motorPower power until 10000 encoder position
            double power = motorPower;
            int targetEncoderValue = 2250;
            DriveDirection direction = DriveDirection.PIVOT_BACKWARD_RIGHT;
            DcMotor.RunMode mode = DcMotor.RunMode.RUN_USING_ENCODER;

            telemetry.addLine("state0");
            telemetry.addData("driveLeft pos: ", robot.driveLeft.getCurrentPosition());
            telemetry.addData("driveRight pos: ", robot.driveRight.getCurrentPosition());
            telemetry.update();
            targetReached =
                    tankDriveToEncoder.runToTarget(power, targetEncoderValue, direction, mode);


            if (targetReached) {
                state0 = false;
                state0_5 = true;
            }
        } else if (state0_5) {

            boolean targetReached = false;
            //drive forward at motorPower power until 10000 encoder position
            double power = motorPower;
            int targetEncoderValue = 12000;
            DriveDirection direction = DriveDirection.DRIVE_BACKWARD;
            DcMotor.RunMode mode = DcMotor.RunMode.RUN_USING_ENCODER;
            telemetry.addLine("state0_5");
            telemetry.update();

            targetReached =
                    tankDriveToEncoder.runToTarget(power, targetEncoderValue, direction, mode);
            if (targetReached) {

                state0_5 = false;
                state0_75 = true;
            }
        } else if (state0_75) {


            boolean targetReached = false;
            //drive forward at .65 power until 10000 encoder position
            double power = .65;
            int targetB = 50;
            int targetR = 50;
            int targetG = 50;
            DriveDirection direction = DriveDirection.DRIVE_BACKWARD;
            telemetry.addLine("state0_75");
            telemetry.update();

            targetReached =
                    tankDriveToColor.runToTarget(power, targetR, targetB, targetG, direction);
            if (targetReached) {

                state0_75 = false;
                state1 = true;
            }
        } else if (state1) {
            DcMotor.RunMode mode = DcMotor.RunMode.RESET_ENCODERS;
            boolean targetReached = false;
            //drive forward at motorPower power until 10000 encoder position
            double power = motorPower;
            int targetEncoderValue = 1200;
            DriveDirection direction;
            if (isBlue) {
                direction = DriveDirection.SPIN_RIGHT;
            } else {
                direction = DriveDirection.SPIN_LEFT;
            }
            mode = DcMotor.RunMode.RUN_USING_ENCODER;
            telemetry.addLine("state1");
            telemetry.update();

            targetReached =
                    tankDriveToEncoder.runToTarget(power, targetEncoderValue, direction, mode);

            if (targetReached) {


                state2 = true;
                state1 = false;
            }
        } else if (state2) {
            boolean targetReached = false;
            //drive forward at motorPower power until 10000 encoder position
            double power = motorPower;
            int targetEncoderValue = 1800;
            DriveDirection direction = DriveDirection.DRIVE_BACKWARD;
            DcMotor.RunMode mode = DcMotor.RunMode.RUN_USING_ENCODER;


            targetReached =
                    tankDriveToEncoder.runToTarget(power, targetEncoderValue, direction, mode);
            telemetry.addLine("state2");
            telemetry.update();

            targetReached =
                    tankDriveToEncoder.targetReached();


            if (targetReached) {


                state3 = true;
                state2 = false;
            }
        } else if (state3) {
            /** TODO: Learn how to read the color sensor **/
            // sleep because the needs to settle on a color.
         //   firstBeaconIsTeamColor = colorSensorComponentLeft.isRed(10, 10, 10) || colorSensorComponentRight.isRed(10, 10, 10) || colorSensorComponent2.isRed(10, 10, 10);

            if (colorSensorComponentLeft.isRed(10, 10, 10)) {
                boolean targetReached = false;
                //drive forward at motorPower power until 10000 encoder position
                double power = motorPower;
                int targetEncoderValue = 800;
                DriveDirection direction = DriveDirection.PIVOT_BACKWARD_RIGHT;
                DcMotor.RunMode mode = DcMotor.RunMode.RUN_USING_ENCODER;


                targetReached =
                        tankDriveToEncoder.runToTarget(power, targetEncoderValue, direction, mode);
                telemetry.addLine("state3");
                telemetry.update();

                targetReached =
                        tankDriveToEncoder.targetReached();


                if (targetReached) {


                    state4 = true;
                    state3 = false;
                }

                else {
                    boolean targetReached2 = false;
                    //drive forward at motorPower power until 10000 encoder position
                    double power2 = motorPower;
                    int targetEncoderValue2 = 800;
                    DriveDirection direction2 = DriveDirection.PIVOT_BACKWARD_LEFT;
                    DcMotor.RunMode mode2 = DcMotor.RunMode.RUN_USING_ENCODER;


                    targetReached2 =
                            tankDriveToEncoder.runToTarget(power2, targetEncoderValue2, direction2, mode2);
                    telemetry.addLine("state3");
                    telemetry.update();

                    targetReached =
                            tankDriveToEncoder.targetReached();


                    if (targetReached) {


                        state4 = true;
                        state3 = false;
                    }
                }

                telemetry.addData("0state3 B: ", colorSensorComponentLeft.getB());
                telemetry.addData("0state3 R: ", colorSensorComponentLeft.getR());
                telemetry.addData("0state3 G: ", colorSensorComponentLeft.getG());

                telemetry.addData("1state3 B: ", colorSensorComponentRight.getB());
                telemetry.addData("1state3 R: ", colorSensorComponentRight.getR());
                telemetry.addData("1state3 G: ", colorSensorComponentRight.getG());

                telemetry.addData("2state3 B: ", colorSensorComponent2.getB());
                telemetry.addData("2state3 R: ", colorSensorComponent2.getR());
                telemetry.addData("2state3 G: ", colorSensorComponent2.getG());


                telemetry.addData("state3 isRed: ", firstBeaconIsTeamColor);
                telemetry.update();

                //firstBeaconIsTeamColor = false;
                // getTelemetryUtil(robot.color);
                //     boolean isRed = colorSensorComponent.isRed(5, 2, 2);
                //   boolean isRed = colorSensorComponent.isRed(10, 2, 2);


                //Detect beacon color


            } else if (state4) {
                boolean targetReached = false;
                //drive forward at motorPower power until 10000 encoder position
                double power = motorPower;
                int targetEncoderValue = 2000;
                DriveDirection direction = DriveDirection.DRIVE_FORWARD;
                DcMotor.RunMode mode = DcMotor.RunMode.RUN_USING_ENCODER;


                targetReached =
                        tankDriveToEncoder.runToTarget(power, targetEncoderValue, direction, mode);

                if (targetReached) {

                    state5 = true;
                    state4 = false;
                }
            } else if (state5) {
                boolean targetReached = false;
                //drive forward at motorPower power until 10000 encoder position
                double power = motorPower;
                int targetEncoderValue = 3300;
                DriveDirection direction;
                if (isBlue) {
                    direction = DriveDirection.SPIN_LEFT;
                } else {
                    direction = DriveDirection.SPIN_RIGHT;
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
                //drive forward at motorPower power until 10000 encoder position
                double power = motorPower;
                int targetEncoderValue = 11000;
                DriveDirection direction = DriveDirection.DRIVE_BACKWARD;
                DcMotor.RunMode mode = DcMotor.RunMode.RUN_USING_ENCODER;


                targetReached =
                        tankDriveToEncoder.runToTarget(power, targetEncoderValue, direction, mode);

                if (targetReached) {

                    state6_5 = true;
                    state6 = false;
                }
            } else if (state6_5) {


                boolean targetReached = false;
                //drive forward at .65 power until 10000 encoder position
                double power = .1;
                int targetB = 50;
                int targetR = 50;
                int targetG = 50;
                DriveDirection direction = DriveDirection.DRIVE_BACKWARD;

                targetReached =
                        tankDriveToColor.runToTarget(power, targetR, targetB, targetG, direction);
                if (targetReached) {

                    state6_5 = false;
                    state7 = true;
                }
            } else if (state7) {
                boolean targetReached = false;
                //drive forward at motorPower power until 10000 encoder position
                double power = motorPower;
                int targetEncoderValue = 2700;
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

                    state8 = true;
                    state7 = false;
                }
            } else if (state8) {
                boolean targetReached = false;
                //drive forward at motorPower power until 10000 encoder position
                double power = motorPower;
                int targetEncoderValue = 4800;
                DriveDirection direction = DriveDirection.DRIVE_BACKWARD;
                DcMotor.RunMode mode = DcMotor.RunMode.RUN_USING_ENCODER;


                targetReached =
                        tankDriveToEncoder.runToTarget(power, targetEncoderValue, direction, mode);

                if (targetReached) {

                    state8 = false;
                    sleep(500);
                    // TODO how to learn to read the color sensor
                    if (colorSensorComponentLeft.isRed(10, 10, 10)) {
                        state9Correct = true;
                    } else {
                        state9InCorrect = true;
                    }
                    telemetry.addData("0state3 B: ", colorSensorComponentLeft.getB());
                    telemetry.addData("0state3 R: ", colorSensorComponentLeft.getR());
                    telemetry.addData("0state3 G: ", colorSensorComponentLeft.getG());

                    telemetry.addData("1state3 B: ", colorSensorComponentRight.getB());
                    telemetry.addData("1state3 R: ", colorSensorComponentRight.getR());
                    telemetry.addData("1state3 G: ", colorSensorComponentRight.getG());

                    telemetry.addData("2state3 B: ", colorSensorComponent2.getB());
                    telemetry.addData("2state3 R: ", colorSensorComponent2.getR());
                    telemetry.addData("2state3 G: ", colorSensorComponent2.getG());


                    telemetry.update();
                }

            } else if (state9InCorrect) {
                boolean targetReached = false;
                //drive forward at motorPower power until 10000 encoder position
                double power = motorPower;
                int targetEncoderValue = 2000;
                DriveDirection direction = DriveDirection.PIVOT_BACKWARD_LEFT;
                DcMotor.RunMode mode = DcMotor.RunMode.RUN_USING_ENCODER;


                targetReached =
                        tankDriveToEncoder.runToTarget(power, targetEncoderValue, direction, mode);

                if (targetReached) {
                    sleep(3000);
                    state9InCorrect = false;
                    state9Correct_5 = true;
                }
            // else if (state9InCorrect_5) {
               // boolean targetReached = false;
                //drive forward at motorPower power until 10000 encoder position
//                //double power = motorPower;
  //              int targetEncoderValue = 2500;
    //            DriveDirection direction = DriveDirection.DRIVE_BACKWARD;
      //          DcMotor.RunMode mode = DcMotor.RunMode.RUN_USING_ENCODER;


//                targetReached =
  //                      tankDriveToEncoder.runToTarget(power, targetEncoderValue, direction, mode);

//                if (targetReached) {

  //                  state9Correct = true;
    //                state9InCorrect_5 = false;

            } else if (state9Correct) {
                boolean targetReached = false;
                //drive forward at motorPower power until 10000 encoder position
                double power = motorPower;
                int targetEncoderValue = 5000;
                DriveDirection direction = DriveDirection.PIVOT_BACKWARD_RIGHT;
                DcMotor.RunMode mode = DcMotor.RunMode.RUN_USING_ENCODER;


                targetReached =
                        tankDriveToEncoder.runToTarget(power, targetEncoderValue, direction, mode);

                if (targetReached) {

                    state9Correct_5 = true;
                    state9Correct = false;
                }
            } else if (state9Correct_5) {
                boolean targetReached = false;
                //drive forward at motorPower power until 10000 encoder position
                double power = motorPower;
                int targetEncoderValue = 2700;
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

                    state9Correct_5 = false;
                    state10 = true;
                }
            } else if (state10) {
                boolean targetReached = false;
                //drive forward at motorPower power until 10000 encoder position
                double power = motorPower;
                int targetEncoderValue = 10000;
                DriveDirection direction = DriveDirection.DRIVE_BACKWARD;

                DcMotor.RunMode mode = DcMotor.RunMode.RUN_USING_ENCODER;


                targetReached =
                        tankDriveToEncoder.runToTarget(power, targetEncoderValue, direction, mode);

                if (targetReached) {


                    state10 = false;
                    state11 = true;


                }
            } else if (state11) {
                boolean targetReached = false;
                //drive forward at motorPower power until 10000 encoder position
                double power = motorPower;
                int targetEncoderValue = 1400;
                DriveDirection direction;
                if (isBlue) {
                    direction = DriveDirection.SPIN_LEFT;
                } else {
                    direction = DriveDirection.SPIN_RIGHT;
                }
                DcMotor.RunMode mode = DcMotor.RunMode.RUN_USING_ENCODER;


                targetReached =
                        tankDriveToEncoder.runToTarget(power, targetEncoderValue, direction, mode);

                if (targetReached) {

                    state11 = false;
                    // TODO how to learn to read the color sensor

                        state12Correct = true;

                }
            } else if (state12InCorrect) {
                boolean targetReached = false;
                //drive forward at motorPower power until 10000 encoder position
                double power = motorPower;
                int targetEncoderValue = 1500;
                DriveDirection direction;
                if (isBlue) {
                    direction = DriveDirection.SPIN_LEFT;
                } else {
                    direction = DriveDirection.SPIN_RIGHT;
                }
                DcMotor.RunMode mode = DcMotor.RunMode.RUN_USING_ENCODER;


                targetReached =
                        tankDriveToEncoder.runToTarget(power, targetEncoderValue, direction, mode);

                if (targetReached) {

                    state12InCorrect = false;
                    state12InCorrect_5 = true;
                }
            } else if (state12InCorrect_5) {
                boolean targetReached = false;
                //drive forward at motorPower power until 10000 encoder position
                double power = motorPower;
                int targetEncoderValue = 9000;
                DriveDirection direction = DriveDirection.DRIVE_BACKWARD;
                DcMotor.RunMode mode = DcMotor.RunMode.RUN_USING_ENCODER;


                targetReached =
                        tankDriveToEncoder.runToTarget(power, targetEncoderValue, direction, mode);
                if (targetReached) {

                    state12InCorrect_75 = true;
                    state12InCorrect_5 = false;
                }

            } else if (state12InCorrect_75) {
                boolean targetReached = false;
                //drive forward at motorPower power until 10000 encoder position
                double power = motorPower;
                int targetEncoderValue = 7000;
                DriveDirection direction = DriveDirection.DRIVE_FORWARD;
                DcMotor.RunMode mode = DcMotor.RunMode.RUN_USING_ENCODER;


                targetReached =
                        tankDriveToEncoder.runToTarget(power, targetEncoderValue, direction, mode);
                if (targetReached) {

                    state12InCorrect_9 = true;
                    state12InCorrect_75 = false;
                }

            } else if (state12InCorrect_9) {
                boolean targetReached = false;
                //drive forward at motorPower power until 10000 encoder position
                double power = motorPower;
                int targetEncoderValue = 1800;
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

                    state12Correct = true;
                    state12InCorrect_9 = false;
                }

            } else if (state12Correct) {
                boolean targetReached = false;
                double power = motorPower;
                int targetEncoderValue = 14000;
                DriveDirection direction = DriveDirection.DRIVE_BACKWARD;
                DcMotor.RunMode mode = DcMotor.RunMode.RUN_USING_ENCODER;

                targetReached =
                        tankDriveToEncoder.runToTarget(power, targetEncoderValue, direction, mode);
                if (targetReached) {
                    state12Correct = false;

                }
            }
        }
    }
}
