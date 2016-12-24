package org.ftcTeam.opmodes;

import org.ftcTeam.opmodes.registrar1.GamePadDriveOpMode;
import org.ftcTeam.opmodes.registrar2.AutonomousBeaconBlue;
import org.ftcTeam.opmodes.registrar2.AutonomousBeaconRed;
import org.ftcTeam.opmodes.registrar2.AutonomousBlue;
import org.ftcTeam.opmodes.registrar2.AutonomousRed;
import org.ftcTeam.opmodes.registrar2.GamePadDriveOpModeWithSpinner;
import org.ftcTeam.opmodes.registrar2.TestAutonomous;
import org.ftcTeam.opmodes.registrar2.TestColorSensor;
import org.ftcTeam.opmodes.registrar2.TestRunToColor;
import org.ftcTeam.opmodes.registrar2.TestSingleColorSensor;
import org.ftcbootstrap.BootstrapRegistrar;
import org.ftcbootstrap.demos.TelemetryTest;


/**
 * Register Op Modes
 */
public class Registrar2 extends BootstrapRegistrar {


  protected Class[] getOpmodeClasses() {
    Class[] classes = {

            GamePadDriveOpMode.class,
            GamePadDriveOpModeWithSpinner.class,
            TelemetryTest.class,
            AutonomousBlue.class,
            TestAutonomous.class,
            AutonomousBeaconRed.class,
            AutonomousBeaconBlue.class,
            AutonomousRed.class,
            TestColorSensor.class,
            TestSingleColorSensor.class,
            TestRunToColor.class




    };

    return classes;

  }
}
