package org.ftcTeam;

import java.util.ArrayList;

/**
 * Created by NovaLabs Robotics on 11/23/2016.
 */

public class State {

    private String name;
    private ArrayList<State> allowedNextStates;

    public State(String _name, ArrayList<State> _nextStatesToAllow) {
        name = _name;
        allowedNextStates = _nextStatesToAllow;
    }
}
