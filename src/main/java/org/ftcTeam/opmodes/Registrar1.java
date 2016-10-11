package org.ftcTeam.opmodes;

import org.ftcTeam.opmodes.registrar1.*;
import org.ftcbootstrap.BootstrapRegistrar;
import org.ftcbootstrap.demos.TelemetryTest;


/**
 * Register Op Modes
 */
public class Registrar1 extends BootstrapRegistrar {


  protected Class[] getOpmodeClasses() {
    Class[] classes = {

            GamePadDriveOpMode.class,
            TestAutonomous.class,
            TelemetryTest.class

    };

    return classes;

  }
}
