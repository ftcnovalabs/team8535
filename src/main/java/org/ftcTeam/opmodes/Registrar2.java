package org.ftcTeam.opmodes;

import org.ftcTeam.opmodes.registrar2.AutonomousBlue;
import org.ftcTeam.opmodes.registrar2.AutonomousRed;
import org.ftcTeam.opmodes.registrar2.GamePadDriveOpModeWithSpinner;
import org.ftcTeam.opmodes.registrar2.TestStuff;
import org.ftcbootstrap.BootstrapRegistrar;
import org.ftcbootstrap.demos.TelemetryTest;


/**
 * Register Op Modes
 */
public class Registrar2 extends BootstrapRegistrar {


  protected Class[] getOpmodeClasses() {
    Class[] classes = {

            GamePadDriveOpModeWithSpinner.class,
            TelemetryTest.class,
            AutonomousBlue.class,
            AutonomousRed.class,
            TestStuff.class,

    };

    return classes;

  }
}
