package org.ftcTeam.opmodes.registrar2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Note:  It is assumed that the proper registrar is used for this set of demos. To confirm please
 * search for "Enter your custom registrar"  in  {@link org.ftcTeam.FTCTeamControllerActivity}
 * <p/>
 */

@Autonomous
public class AutonomousBeaconRed extends AutonomousBeacon{

    @Override
    protected boolean isRedAlliance() {
        return true;
    }

}
