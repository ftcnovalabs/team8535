package org.ftcTeam;

import java.util.ArrayList;

/**
 * Created by NovaLabs Robotics on 11/23/2016.
 */

public class State {

    public enum DESCRIPTION {
        INITIAL, BEACON1_ALIGNED, BEACON1_FACING, BEACON1_PRESSED
    }

    private DESCRIPTION name;
    private ArrayList<State> allowedNextStates;

    public State(State.DESCRIPTION _name, ArrayList<State> nextStatesToAllow) {
        name = _name;
        allowedNextStates = nextStatesToAllow;
    }
}
