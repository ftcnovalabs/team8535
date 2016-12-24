package org.ftcTeam.opmodes.registrar2;

import org.ftcTeam.opmodes.registrar2.GamePadDriveOpModeWithSpinner;
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
            AutonomousRed.class

    };

    return classes;

  }
}
