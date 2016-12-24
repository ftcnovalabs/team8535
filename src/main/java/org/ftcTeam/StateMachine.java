package org.ftcTeam;


import java.util.EnumMap;

/**
 * Created by NovaLabs Robotics on 11/14/2016.
 */

public class StateMachine {

    /**
     * // Initial state (robot at start, not moving)
     * private boolean state0 = true;
     * // Aligned with First Beacon
     * private boolean state1 = false;
     * // Turned left to face first beacon
     * private boolean state2 = false;
     * //press first beacon button(s)
     * private boolean state3 = false;
     * // Check Color Sensor then store color01 value
     * private boolean state4 = false;
     * //knock of Cap ball
     * private boolean state5 = false;
     * //face second beacon
     * private boolean state6 = false;
     * //approach second beacon
     * private boolean state7 = false;
     * //face second beacon
     * private boolean state8 = false;
     * //press second beacon button(s)
     * private boolean state9Correct = false;
     * // Check Color Sensor then store color01 value
     * private boolean state9InCorrect = false;
     * //Drive backward wait 5 seconds then drive forward to switch beacon color01
     * private boolean state10 = false;
     * //clear robot to turn
     * private boolean state11 = false;
     * //prepare to travel to first beacon
     * private boolean state12InCorrect = false;
     * //align with first beacon and press first beacon button(s)
     * private boolean state12Correct = false;
     * // Clear robot to turn
     * private boolean state13 = false;
     * // face corner vortex
     * private boolean state14 = false;
     * //park on corner vortex
     **/

    public boolean beacon1Correct;
    public boolean beacon2Correct;



    //public STATE currentState;

    //private EnumMap<STATE,Integer> stateMap;

    public StateMachine(boolean isBlue){
    /*    stateMap = new EnumMap<STATE, Integer>(STATE.class);
        stateMap.put(STATE.INITIAL,0);
        stateMap.put(STATE.BEACON1_ALIGNED,10);
        stateMap.put(STATE.BEACON1_FACING,20);
        stateMap.put(STATE.BEACON1_PRESSED,30);*/


    }


}
