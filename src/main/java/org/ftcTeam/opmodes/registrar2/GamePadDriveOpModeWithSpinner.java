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
    private GamePadMotor motor3;
  //  private GamePadServo clamp_left;
    //private GamePadServo clamp_right;


    //public GamePadDriveOpModeWithSpinner(DcMotor motor3) {
    //    motor3 = null;
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
        motor3 = new GamePadMotor(this, gamepad1, robot.motor3, GamePadMotor.Control.LB_RB_BUTTONS, 2.0f);
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
        motor3.update();
       // clamp_left.update();
     //   clamp_right.update();

        //send any telemetry that may have been added in the above operations
        getTelemetryUtil().sendTelemetry();


    }
}

