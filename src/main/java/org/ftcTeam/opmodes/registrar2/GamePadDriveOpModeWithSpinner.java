package org.ftcTeam.opmodes.registrar2;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.ftcTeam.configurations.FTCTeamRobot;
import org.ftcTeam.configurations.Team7394;
import org.ftcbootstrap.ActiveOpMode;
import org.ftcbootstrap.components.operations.motors.GamePadMotor;
import org.ftcbootstrap.components.operations.motors.GamePadTankDrive;
import org.ftcbootstrap.components.operations.servos.GamePadServo;


/**
 * Note:  It is assumed that the proper registry is used for this set of demos. To confirm please
 * search for "Enter your custom registry here"  in  {@link org.ftcTeam.FTCTeamControllerActivity}
 * <p/>
 * Summary:  Use an Operation class to perform a tank drive using the gamepad joysticks.
 * See: {@link GamePadTankDrive}
 */

@TeleOp
public class GamePadDriveOpModeWithSpinner extends ActiveOpMode {

    private FTCTeamRobot robot;
    private GamePadTankDrive gamePadTankDrive;
    private GamePadMotor spinner;
    private GamePadMotor scissorLeft;
    private GamePadMotor scissorRight;
    private GamePadMotor forkLift;

  //  private GamePadServo clamp_left;
    //private GamePadServo clamp_right;


    //public GamePadDriveOpModeWithSpinner(DcMotor spinner) {
    //    spinner = null;
    //}

    /**
     * Implement this method to define the code to run when the Init button is pressed on the Driver station.
     */
    @Override
    protected void onInit() {

        robot = FTCTeamRobot.newConfig(hardwareMap, getTelemetryUtil());

        //Note The Telemetry Utility is designed to let you organize all telemetry data before sending it to
        //the Driver station via the sendTelemetry command
        getTelemetryUtil().addData("Init", getClass().getSimpleName() + " initialized.");
        getTelemetryUtil().sendTelemetry();

    }

    @Override
    protected void onStart() throws InterruptedException {
        super.onStart();

        //create the operation  to perform a tank drive using the gamepad joysticks.
       gamePadTankDrive = new GamePadTankDrive(this, gamepad1, robot.motor1, robot.motor2);
        spinner = new GamePadMotor(this, gamepad1, robot.spinner, GamePadMotor.Control.LB_RB_BUTTONS, 2.0f);
        scissorLeft = new GamePadMotor(this, gamepad2, robot.scissorLeft, GamePadMotor.Control.LEFT_STICK_Y, 2.0f);
        scissorRight = new GamePadMotor(this, gamepad2, robot.scissorRight, GamePadMotor.Control.LEFT_STICK_Y, 2.0f);
        forkLift = new GamePadMotor(this, gamepad2, robot.forkLift, GamePadMotor.Control.RIGHT_STICK_X, 2.0f);

        //     clamp_left = new GamePadServo(this, gamepad2, robot.clamp_left, GamePadServo.Control.LB_LT_TRIGGER, 1.0f);
     //   clamp_right = new GamePadServo(this, gamepad2, robot.clamp_right, GamePadServo.Control.RB_RT_TRIGGER, 1.0f);

    }

    /**
     * Implement this method to define the code to run when the Start button is pressed on the Driver station.
     * This method will be called on each hardware cycle just as the loop() method is called for event based Opmodes
     *
     * @throws InterruptedException
     */
    @Override
    protected void activeLoop() throws InterruptedException {

        //update the motors with the gamepad joystick values
        gamePadTankDrive.update();
        spinner.update();
        scissorLeft.update();
        scissorRight.update();
        forkLift.update();

        // clamp_left.update();
     //   clamp_right.update();

        //send any telemetry that may have been added in the above operations
        getTelemetryUtil().sendTelemetry();


    }
}

